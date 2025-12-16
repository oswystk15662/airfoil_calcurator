# main_optimizer.py
import numpy as np
import optuna
import os
import glob
import time
from datetime import datetime
from scipy.special import comb 
from bemt_solver.geometry import Propeller
from bemt_solver.core import solve_bemt
from airfoil_database_airfoiltools import get_available_airfoils, get_airfoil_properties

# --- 1. 設計の基本パラメータ (Telloのスペック) ---
DIAMETER = 0.076  # 76 mm
TIP_RADIUS = DIAMETER / 2.0
RPM = 15000.0
V_INFINITY = 0.0 
AIR_DENSITY = 1.225
KINEMATIC_VISCOSITY = 1.4607e-5

# --- 2. 計算精度と制御点の定義 ---
NUM_BEMT_ELEMENTS = 20
NUM_GEOM_CONTROL_POINTS = 5
NUM_AIRFOIL_CONTROL_POINTS = 4

# --- 3. 最適化の制約 ---
TARGET_POWER_LIMIT = 3.26  # (W)
TARGET_THRUST_MIN = 0.196  # (N)

# [重要] ハブ径の制約
# インフィル100%対策のため極小ハブを目指す
# シャフト穴(0.79mm) + 肉厚(1mm) = 半径1.5mm (直径3mm) は最低限必要と仮定
MIN_HUB_RADIUS_M = 0.0015 # 1.5mm (直径3.0mm) 
MAX_HUB_DIAMETER_MM = 3.79 # 最大直径 3.79mm

# その他の制約
MAX_DUCT_LIP_RADIUS_M = 0.010 
MIN_ABSOLUTE_THICKNESS_M = 0.0003 

# --- 4. 最適化の探索空間 ---
AIRFOIL_CHOICES = get_available_airfoils()
if not AIRFOIL_CHOICES:
    raise RuntimeError("エアフォイルデータベースが空です。先に generate_database.py を実行してください。")

SPAN_POSITIONS_BEMT = np.linspace(0.0, 1.0, NUM_BEMT_ELEMENTS)
SPAN_POSITIONS_AIRFOIL = np.array([0.0, 0.5, 1.0]) 

# (Bézier関数)
def _bernstein_polynomial(i, n, t):
    return comb(n, i) * (t**i) * ((1 - t)**(n - i))

def generate_bezier_distribution(control_points_y, num_output_points):
    n = len(control_points_y) - 1
    t = np.linspace(0, 1, num_output_points)
    curve = np.zeros(num_output_points)
    for i in range(n + 1):
        curve += control_points_y[i] * _bernstein_polynomial(i, n, t)
    return curve

# --- 6. Optuna 目的関数 ---

def evaluate_design(trial):
    """ Optunaが呼び出す目的関数 """
    
    # --- 1. グローバル変数の提案 ---
    num_blades = trial.suggest_int("num_blades", 2, 5) 
    
    # [修正] ハブ比率の範囲計算 (0.1のリミッターを撤廃)
    min_hub_ratio = MIN_HUB_RADIUS_M / TIP_RADIUS
    
    # 最大ハブ比率
    max_hub_ratio_limit = (MAX_HUB_DIAMETER_MM / 1000.0 / 2.0) / TIP_RADIUS
    
    # 矛盾チェック: もし最大値設定が最小値を下回っていたら、最小値+0.01で探索
    if max_hub_ratio_limit <= min_hub_ratio:
        # print(f"Warning: MAX_HUB_DIAMETER ({MAX_HUB_DIAMETER_MM}mm) is too small for MIN_HUB_RADIUS ({MIN_HUB_RADIUS_M*1000:.1f}mm). Adjusting.")
        max_hub_ratio_limit = min_hub_ratio + 0.01
        
    hub_ratio = trial.suggest_float("hub_ratio", min_hub_ratio, max_hub_ratio_limit)
    
    # --- ダクト形状の制約 ---
    duct_len = trial.suggest_float("duct_length", 0.0, TIP_RADIUS)
    if duct_len < 1e-6:
        duct_lip = 0.0
    else:
        max_possible_lip_radius = min(MAX_DUCT_LIP_RADIUS_M, duct_len) 
        duct_lip = trial.suggest_float("duct_lip_radius", 0.0, max_possible_lip_radius)

    hub_radius = TIP_RADIUS * hub_ratio
    blade_span = TIP_RADIUS - hub_radius
    r_coords_bemt = hub_radius + SPAN_POSITIONS_BEMT * blade_span
    
    # --- 2. 翼型 ---
    airfoil_names = [
        trial.suggest_categorical("airfoil_0_hub", AIRFOIL_CHOICES),
        trial.suggest_categorical("airfoil_1_mid", AIRFOIL_CHOICES),
        trial.suggest_categorical("airfoil_2_tip", AIRFOIL_CHOICES)
    ]
    r_coords_airfoil_def = hub_radius + SPAN_POSITIONS_AIRFOIL * blade_span

    # --- 3. 弦長 ---
    chord_control_points_y = [
        trial.suggest_float(f"chord_ctrl_0", 0.003, 0.005, step=0.0001), 
        trial.suggest_float(f"chord_ctrl_1", 0.004, 0.005, step=0.0001),
        trial.suggest_float(f"chord_ctrl_2", 0.003, 0.005, step=0.0001),
        trial.suggest_float(f"chord_ctrl_3", 0.002, 0.005, step=0.0001),
        trial.suggest_float(f"chord_ctrl_4", 0.002, 0.004, step=0.0001)
    ]

    # --- 4. ピッチ角 ---
    pitch_control_points_y = [
        trial.suggest_float(f"pitch_ctrl_0", 15.0, 35.0),
        trial.suggest_float(f"pitch_ctrl_1", 12.0, 30.0),
        trial.suggest_float(f"pitch_ctrl_2", 10.0, 25.0),
        trial.suggest_float(f"pitch_ctrl_3", 5.0, 20.0),
        trial.suggest_float(f"pitch_ctrl_4", 5.0, 18.0)
    ]

    # --- 5. 分布生成 ---
    pitch_distribution = generate_bezier_distribution(pitch_control_points_y, NUM_BEMT_ELEMENTS)
    chord_distribution = generate_bezier_distribution(chord_control_points_y, NUM_BEMT_ELEMENTS)

    # --- 6. 制約チェック ---
    idx_map = np.argmin(np.abs(r_coords_airfoil_def[:, None] - r_coords_bemt), axis=0)
    for i in range(NUM_BEMT_ELEMENTS):
        airfoil_name = airfoil_names[idx_map[i]]
        _, _, t_c_ratio = get_airfoil_properties(airfoil_name, 10000, 0)
        actual_thickness_m = chord_distribution[i] * t_c_ratio
        if actual_thickness_m < MIN_ABSOLUTE_THICKNESS_M: 
            return -9999.0
            
    # --- 7. 性能評価 ---
    prop = Propeller(
        hub_radius=hub_radius,
        tip_radius=TIP_RADIUS,
        num_blades=num_blades,
        r_coords=r_coords_bemt,
        pitch_coords_deg=pitch_distribution,
        chord_coords=chord_distribution,
        r_coords_airfoil_def=r_coords_airfoil_def,
        airfoil_names=airfoil_names,
        duct_length=duct_len,
        duct_lip_radius=duct_lip
    )
    
    (total_T, _, _, _, P, _) = solve_bemt(
        prop, V_INFINITY, RPM, AIR_DENSITY, KINEMATIC_VISCOSITY, num_elements=NUM_BEMT_ELEMENTS
    )

    if P > TARGET_POWER_LIMIT:
        return TARGET_THRUST_MIN - (P - TARGET_POWER_LIMIT) 
    if total_T < TARGET_THRUST_MIN:
        return total_T 
    
    return total_T

# --- 実行ブロック ---
if __name__ == "__main__":
    
    output_lines = []
    def log_and_print(message):
        print(message)
        output_lines.append(str(message))
    
    # 探索範囲の計算結果を表示
    actual_min_hub_ratio = MIN_HUB_RADIUS_M / TIP_RADIUS
    actual_max_hub_ratio = (MAX_HUB_DIAMETER_MM / 1000.0 / 2.0) / TIP_RADIUS
    if actual_max_hub_ratio < actual_min_hub_ratio:
        actual_max_hub_ratio = actual_min_hub_ratio + 0.01

    log_and_print("--- 🛠️  Step 5: Optimized Propeller Design (Hub Restricted v2) ---")
    log_and_print(f"Target: Maximize Thrust @ {RPM} RPM")
    log_and_print(f"Constraints: Power <= {TARGET_POWER_LIMIT} W, Thrust >= {TARGET_THRUST_MIN} N")
    log_and_print(f"             Hub Dia: {MIN_HUB_RADIUS_M*2000:.1f}mm - {MAX_HUB_DIAMETER_MM:.1f}mm")
    log_and_print(f"             (Ratio: {actual_min_hub_ratio:.3f} - {actual_max_hub_ratio:.3f})")
    log_and_print("--------------------------------------------------")

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(direction="maximize")
    
    n_trials = 500
    log_and_print(f"Running Optuna ({n_trials} trials)...")
    start_time = time.time()
    study.optimize(evaluate_design, n_trials=n_trials, n_jobs=-1)
    end_time = time.time()

    log_and_print(f"\nOptimization finished in {end_time - start_time:.2f} seconds.")
    
    if study.best_trial.value < TARGET_THRUST_MIN:
        log_and_print("❌ Optimization FAILED to meet minimum thrust constraint.")
        log_and_print(f"   (Best attempt value: {study.best_trial.value:.4f})")
    else:
        log_and_print("✅ Best solution found:")
        best_trial = study.best_trial
        
        log_and_print(f"  Best Thrust: {best_trial.value:.4f} N")
        
        # 結果表示 (省略なし)
        log_and_print("\n  Optimal Parameters (Global):")
        log_and_print(f"    num_blades: {best_trial.params['num_blades']}")
        log_and_print(f"    hub_ratio: {best_trial.params['hub_ratio']:.3f} (Dia: {best_trial.params['hub_ratio']*DIAMETER*1000:.1f} mm)")
        log_and_print(f"    duct_length: {best_trial.params['duct_length']*1000:.1f} mm")
        log_and_print(f"    duct_lip_radius: {best_trial.params['duct_lip_radius']*1000:.1f} mm")
        
        best_params = best_trial.params
        best_hub_ratio = best_params["hub_ratio"]
        best_hub_radius = TIP_RADIUS * best_hub_ratio
        
        # ... (制御点やCADデータの表示は前と同じ) ...
        # (長くなるのでここは前のコードと同じロジックを使ってください)
        # もし必要なら全文記述します
        
        # --- 簡易版CADデータ表示 (全文貼り付け用) ---
        r_coords_bemt = best_hub_radius + SPAN_POSITIONS_BEMT * (TIP_RADIUS - best_hub_radius)
        r_coords_airfoil = best_hub_radius + SPAN_POSITIONS_AIRFOIL * (TIP_RADIUS - best_hub_radius)
        
        pitch_ctrl_points = [best_params[f"pitch_ctrl_{i}"] for i in range(NUM_GEOM_CONTROL_POINTS)]
        chord_ctrl_points = [best_params[f"chord_ctrl_{i}"] for i in range(NUM_GEOM_CONTROL_POINTS)]
        
        pitch_distribution = generate_bezier_distribution(pitch_ctrl_points, NUM_BEMT_ELEMENTS)
        chord_distribution = generate_bezier_distribution(chord_ctrl_points, NUM_BEMT_ELEMENTS)
        
        airfoil_ctrl_names = [
            best_params["airfoil_0_hub"],
            best_params["airfoil_1_mid"],
            best_params["airfoil_2_tip"]
        ]
        
        prop_final = Propeller(
            hub_radius=best_hub_radius,
            tip_radius=TIP_RADIUS,
            num_blades=best_params["num_blades"],
            r_coords=r_coords_bemt,
            pitch_coords_deg=pitch_distribution,
            chord_coords=chord_distribution,
            r_coords_airfoil_def=r_coords_airfoil,
            airfoil_names=airfoil_ctrl_names,
            duct_length=best_params["duct_length"],
            duct_lip_radius=best_params["duct_lip_radius"]
        )
        
        log_and_print("\n--- CAD Data (BEMT Points Definition) ---")
        log_and_print(f"    i | Radius (m) | Pitch (deg) | Chord (mm) | Nearest Airfoil | Abs Thick (mm)")
        log_and_print("    --|------------|-------------|------------|-----------------|---------------")
        
        airfoil_names_final = [prop_final.get_airfoil_name(r) for r in r_coords_bemt]
        for i in range(NUM_BEMT_ELEMENTS):
            _, _, t_c = get_airfoil_properties(airfoil_names_final[i], 10000, 0)
            abs_thick_mm = chord_distribution[i] * t_c * 1000
            log_and_print(f"    {i:2d} |   {r_coords_bemt[i]:.4f}   |   {pitch_distribution[i]:8.3f} |   {chord_distribution[i]*1000:6.1f}   | {airfoil_names_final[i]:<15} |    {abs_thick_mm:.2f}")

    # 保存
    output_dir = "./optimization_results"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    timestamp = datetime.now().strftime("%m%d%H%M")
    filename = os.path.join(output_dir, f"result_{timestamp}.txt")
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("\n".join(output_lines))
        print(f"\n✅ Results saved to {filename}")
    except Exception as e:
        print(f"\n❌ Error saving results to file: {e}")

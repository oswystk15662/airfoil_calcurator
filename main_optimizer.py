# main_optimizer.py (全面差し替え)
import numpy as np
import optuna
import os
import glob
import time
from datetime import datetime
from scipy.special import comb 
from bemt_solver.geometry import Propeller
from bemt_solver.core import solve_bemt
from airfoil_database_airfoiltools import get_available_airfoils

# --- 1. 設計の基本パラメータ (Telloのスペック) ---
DIAMETER = 0.076  # 76 mm
TIP_RADIUS = DIAMETER / 2.0
RPM = 15000.0
V_INFINITY = 0.0 # ホバー時の最適化
AIR_DENSITY = 1.225
KINEMATIC_VISCOSITY = 1.4607e-5

# --- 2. 計算精度と制御点の定義 ---
NUM_BEMT_ELEMENTS = 20
NUM_GEOM_CONTROL_POINTS = 5
NUM_AIRFOIL_CONTROL_POINTS = 3

# --- 3. 最適化の制約 ---
TARGET_POWER_LIMIT = 3.26  # (W)
TARGET_THRUST_MIN = 0.196  # (N)
MIN_HUB_RADIUS_M = 0.005 # (m)

# --- 4. 最適化の探索空間 ---
AIRFOIL_CHOICES = get_available_airfoils()
if not AIRFOIL_CHOICES:
    raise RuntimeError("エアフォイルデータベースが空です。先に generate_database.py を実行してください。")

SPAN_POSITIONS_BEMT = np.linspace(0.0, 1.0, NUM_BEMT_ELEMENTS)
SPAN_POSITIONS_AIRFOIL = np.array([0.0, 0.5, 1.0]) 

# (Bézier曲線ヘルパー関数 ... _bernstein_polynomial, generate_bezier_distribution ... は前回のコードと同じ)
def _bernstein_polynomial(i, n, t):
    """ ベルンシュタイン基底関数 """
    return comb(n, i) * (t**i) * ((1 - t)**(n - i))

def generate_bezier_distribution(control_points_y, num_output_points):
    """ 制御点のY座標リストからBézier曲線上のY座標分布を返す """
    n = len(control_points_y) - 1
    t = np.linspace(0, 1, num_output_points)
    curve = np.zeros(num_output_points)
    for i in range(n + 1):
        curve += control_points_y[i] * _bernstein_polynomial(i, n, t)
    return curve

# --- 6. Optuna 目的関数 ---

def evaluate_design(trial):
    """ Optunaが呼び出す目的関数 (ダクト形状も最適化) """
    
    # --- 1. グローバル変数の提案 ---
    num_blades = trial.suggest_int("num_blades", 2, 5) 
    
    min_hub_ratio = max(0.1, MIN_HUB_RADIUS_M / TIP_RADIUS)
    hub_ratio = trial.suggest_float("hub_ratio", min_hub_ratio, 0.30)
    
    # --- 🔽 [修正] ダクト形状の最適化変数を追加 🔽 ---
    # ダクト長さ (0m = ダクトなし ～ チップ半径 38mm まで)
    duct_len = trial.suggest_float("duct_length", 0.0, TIP_RADIUS)
    
    # リップ半径 (0mm ～ 10mm まで)
    # (d/D=0.031  は約 2.4mm なので、10mm あれば十分)
    duct_lip = trial.suggest_float("duct_lip_radius", 0.0, 0.010)
    # --- 🔼 [修正完了] 🔼

    hub_radius = TIP_RADIUS * hub_ratio
    blade_span = TIP_RADIUS - hub_radius
    r_coords = hub_radius + SPAN_POSITIONS_BEMT * blade_span
    
    # --- 2. 翼型 (3点の制御点) ---
    airfoil_names = [
        trial.suggest_categorical("airfoil_0_hub", AIRFOIL_CHOICES),
        trial.suggest_categorical("airfoil_1_mid", AIRFOIL_CHOICES),
        trial.suggest_categorical("airfoil_2_tip", AIRFOIL_CHOICES)
    ]
    r_coords_airfoil_def = hub_radius + SPAN_POSITIONS_AIRFOIL * blade_span

    # --- 3. 弦長 (5点の制御点) ---
    chord_control_points_y = [
        trial.suggest_float(f"chord_ctrl_0", 0.003, 0.005, step=0.0001), 
        trial.suggest_float(f"chord_ctrl_1", 0.004, 0.005, step=0.0001),
        trial.suggest_float(f"chord_ctrl_2", 0.003, 0.005, step=0.0001),
        trial.suggest_float(f"chord_ctrl_3", 0.002, 0.005, step=0.0001),
        trial.suggest_float(f"chord_ctrl_4", 0.002, 0.004, step=0.0001)
    ]

    # --- 4. ピッチ角 (5点の制御点) ---
    pitch_control_points_y = [
        trial.suggest_float(f"pitch_ctrl_0", 15.0, 35.0),
        trial.suggest_float(f"pitch_ctrl_1", 12.0, 30.0),
        trial.suggest_float(f"pitch_ctrl_2", 10.0, 25.0),
        trial.suggest_float(f"pitch_ctrl_3", 5.0, 20.0),
        trial.suggest_float(f"pitch_ctrl_4", 5.0, 18.0)
    ]

    # --- 5. BEMTソルバー用の滑らかな分布を生成 ---
    pitch_distribution = generate_bezier_distribution(pitch_control_points_y, NUM_BEMT_ELEMENTS)
    chord_distribution = generate_bezier_distribution(chord_control_points_y, NUM_BEMT_ELEMENTS)

    # --- 6. 性能評価 ---
    prop = Propeller(
        hub_radius=hub_radius,
        tip_radius=TIP_RADIUS,
        num_blades=num_blades,
        r_coords=r_coords,
        pitch_coords_deg=pitch_distribution,
        chord_coords=chord_distribution,
        r_coords_airfoil_def=r_coords_airfoil_def,
        airfoil_names=airfoil_names,
        duct_length=duct_len,       # ◀ [修正] 最適化された値を渡す
        duct_lip_radius=duct_lip    # ◀ [修正] 最適化された値を渡す
    )
    
    (total_T, _, _, 
     _, P, _) = solve_bemt(
        prop, 
        v_infinity=V_INFINITY, 
        rpm=RPM, 
        air_density=AIR_DENSITY, 
        kinematic_viscosity=KINEMATIC_VISCOSITY,
        num_elements=NUM_BEMT_ELEMENTS
    )

    # --- 7. 制約条件の判定 ---
    if P > TARGET_POWER_LIMIT:
        return TARGET_THRUST_MIN - (P - TARGET_POWER_LIMIT) 
    if total_T < TARGET_THRUST_MIN:
        return total_T 
    
    return total_T

# --- 7. 実行ブロック ---
if __name__ == "__main__":
    
    output_lines = []
    def log_and_print(message):
        print(message)
        output_lines.append(str(message))
    
    log_and_print("--- 🛠️  Step 5: Integrated Duct + Propeller Optimization ---")
    log_and_print(f"Target: Maximize Thrust @ {RPM} RPM (Hover)")
    log_and_print(f"Constraints: Power <= {TARGET_POWER_LIMIT} W, Thrust >= {TARGET_THRUST_MIN} N")
    log_and_print(f"Optimizing: Blade(2-5), Hub(10-30%), Duct Length, Duct Lip")
    log_and_print(f"  + Pitch/Chord (Bézier, {NUM_GEOM_CONTROL_POINTS} ctrl pts)")
    log_and_print(f"  + Airfoils ({NUM_AIRFOIL_CONTROL_POINTS} ctrl pts)")
    log_and_print(f"Available Airfoils ({len(AIRFOIL_CHOICES)}): {AIRFOIL_CHOICES}")
    log_and_print("--------------------------------------------------")

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    
    study = optuna.create_study(direction="maximize")
    
    # 制御点(13) + グローバル(2) + ダクト(2) = 17変数
    n_trials = 500 
    log_and_print(f"Running Optuna ({n_trials} trials)...")
    start_time = time.time()
    study.optimize(evaluate_design, n_trials=n_trials)
    end_time = time.time()

    log_and_print(f"\nOptimization finished in {end_time - start_time:.2f} seconds.")
    log_and_print("--------------------------------------------------")
    
    if study.best_trial.value < TARGET_THRUST_MIN:
        log_and_print("❌ Optimization FAILED to meet minimum thrust constraint.")
        log_and_print(f"   Best attempt achieved: {study.best_trial.value:.4f} N")
        
    else:
        log_and_print("✅ Best solution found:")
        best_trial = study.best_trial
        
        log_and_print(f"  Best Thrust: {best_trial.value:.4f} N")
        
        log_and_print("\n  Optimal Parameters (Global):")
        log_and_print(f"    num_blades: {best_trial.params['num_blades']}")
        log_and_print(f"    hub_ratio: {best_trial.params['hub_ratio']:.3f}")
        log_and_print(f"    duct_length: {best_trial.params['duct_length']*1000:.1f} mm")
        log_and_print(f"    duct_lip_radius: {best_trial.params['duct_lip_radius']*1000:.1f} mm")
        
        # --- 制御点の結果を取得・表示 ---
        best_params = best_trial.params
        best_hub_ratio = best_params["hub_ratio"]
        best_hub_radius = TIP_RADIUS * best_hub_ratio
        best_num_blades = best_params["num_blades"]
        best_duct_len = best_params["duct_length"]
        best_duct_lip = best_params["duct_lip_radius"]
        
        log_and_print("\n  Optimal Control Points (Airfoil):")
        airfoil_ctrl_names = [
            best_params["airfoil_0_hub"],
            best_params["airfoil_1_mid"],
            best_params["airfoil_2_tip"]
        ]
        log_and_print(f"    {airfoil_ctrl_names}")

        log_and_print("\n  Optimal Control Points (Pitch, deg):")
        pitch_ctrl_points = [best_params[f"pitch_ctrl_{i}"] for i in range(NUM_GEOM_CONTROL_POINTS)]
        log_and_print(f"    {[round(p, 2) for p in pitch_ctrl_points]}")

        log_and_print("\n  Optimal Control Points (Chord, mm):")
        chord_ctrl_points = [best_params[f"chord_ctrl_{i}"] for i in range(NUM_GEOM_CONTROL_POINTS)]
        log_and_print(f"    {[round(c * 1000, 2) for c in chord_ctrl_points]}")
        
        # --- 最終性能チェック (Bézier曲線で再生成) ---
        r_coords_bemt = best_hub_radius + SPAN_POSITIONS_BEMT * (TIP_RADIUS - best_hub_radius)
        r_coords_airfoil = best_hub_radius + SPAN_POSITIONS_AIRFOIL * (TIP_RADIUS - best_hub_radius)
        
        pitch_distribution = generate_bezier_distribution(pitch_ctrl_points, NUM_BEMT_ELEMENTS)
        
        # 🔽 [バグ修正 2] 弦長の制御点 (chord_ctrl_points) を使用する 🔽
        chord_distribution = generate_bezier_distribution(chord_ctrl_points, NUM_BEMT_ELEMENTS)
        # 🔼 [バグ修正 2] 🔼
        
        prop_final = Propeller(
            hub_radius=best_hub_radius,
            tip_radius=TIP_RADIUS,
            num_blades=best_num_blades,
            r_coords=r_coords_bemt,
            pitch_coords_deg=pitch_distribution,
            chord_coords=chord_distribution, # ◀ [バグ修正 1] 'coords=' を 'chord_coords=' に修正
            r_coords_airfoil_def=r_coords_airfoil,
            airfoil_names=airfoil_ctrl_names,
            duct_length=best_duct_len,
            duct_lip_radius=best_duct_lip
        )
        
        (T_final, T_fan_final, T_duct_final, 
         _, P_final, _) = solve_bemt(
            prop_final, V_INFINITY, RPM, AIR_DENSITY, KINEMATIC_VISCOSITY,
            num_elements=NUM_BEMT_ELEMENTS
        )
        
        log_and_print("\n  Final Performance Check (using smoothed curves):")
        log_and_print(f"    Total Thrust: {T_final:.4f} N")
        log_and_print(f"    (Fan Thrust:  {T_fan_final:.4f} N)")
        log_and_print(f"    (Duct Thrust: {T_duct_final:.4f} N)")
        log_and_print(f"    Power:        {P_final:.2f} W (Constraint: <= {TARGET_POWER_LIMIT} W)")
        log_and_print(f"    g/W:          {(T_final / 9.81 * 1000) / P_final:.2f}")

        # --- 最終的なCAD用データ (BEMT分割数) ---
        log_and_print("\n--- CAD Data (BEMT Points Definition) ---")
        log_and_print(f"    (Total {NUM_BEMT_ELEMENTS} points, i=0 is Hub, i={NUM_BEMT_ELEMENTS-1} is Tip)")
        log_and_print("    i | Radius (m) | Pitch (deg) | Chord (mm) | Nearest Airfoil")
        log_and_print("    --|------------|-------------|------------|----------------")
        
        airfoil_names_final = [prop_final.get_airfoil_name(r) for r in r_coords_bemt]
        
        for i in range(NUM_BEMT_ELEMENTS):
            log_and_print(f"    {i:2d} |   {r_coords_bemt[i]:.4f}   |   {pitch_distribution[i]:8.3f} |   {chord_distribution[i]*1000:6.1f}   | {airfoil_names_final[i]}")

    # --- ファイルへの書き込み ---
    timestamp = datetime.now().strftime("%m%d%H%M")
    filename = f"./optimization_results/result_{timestamp}.txt"
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("\n".join(output_lines))
        print(f"\n✅ Results saved to {filename}")
    except Exception as e:
        print(f"\n❌ Error saving results to file: {e}")

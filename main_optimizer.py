import numpy as np
import optuna
import os
import time
from datetime import datetime
from scipy.special import comb 

# 自作モジュール
from bemt_solver.geometry import Propeller
from bemt_solver.core import solve_bemt
from airfoil_database_airfoiltools import get_available_airfoils, get_airfoil_properties
from config_loader import load_config

# --- 1. 設定の読み込み ---
config = load_config()

# --- 2. 定数の展開 (可読性のためローカル変数に展開) ---
# Drone Specs
DRONE_CONF = config['drone']
DIAMETER = DRONE_CONF['diameter_mm'] / 1000.0
TIP_RADIUS = DIAMETER / 2.0
RPM = float(DRONE_CONF['rpm'])
V_INFINITY = float(DRONE_CONF['v_infinity'])
AIR_DENSITY = float(DRONE_CONF['air_density'])
KINEMATIC_VISCOSITY = float(DRONE_CONF['kinematic_viscosity'])

# Constraints
CONST_CONF = config['constraints']
TARGET_POWER_LIMIT = float(CONST_CONF['max_power_w'])
TARGET_THRUST_MIN = float(CONST_CONF['min_thrust_n'])
MIN_HUB_RADIUS_M = float(CONST_CONF['hub']['min_radius_mm']) / 1000.0
MAX_HUB_DIAMETER_M = float(CONST_CONF['hub']['max_diameter_mm']) / 1000.0
MAX_DUCT_LIP_RADIUS_M = float(CONST_CONF['geometry']['max_duct_lip_mm']) / 1000.0
MIN_ABSOLUTE_THICKNESS_M = float(CONST_CONF['geometry']['min_thickness_mm']) / 1000.0

# Solver Settings
SOLVER_CONF = config['solver']
NUM_BEMT_ELEMENTS = int(SOLVER_CONF['bemt_elements'])
NUM_GEOM_CONTROL_POINTS = int(SOLVER_CONF['geom_control_points'])

# Design Space
DESIGN_SPACE = config['design_space']

# --- 3. 共通計算ロジック ---
SPAN_POSITIONS_BEMT = np.linspace(0.0, 1.0, NUM_BEMT_ELEMENTS)
SPAN_POSITIONS_AIRFOIL = np.array([0.0, 0.5, 1.0]) 

# 翼型リストの取得
AIRFOIL_CHOICES = get_available_airfoils()
if not AIRFOIL_CHOICES:
    raise RuntimeError("エアフォイルデータベースが空です。generate_database.py を実行してください。")

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

# --- 4. Optuna 目的関数 ---

def evaluate_design(trial):
    """ YAML設定に基づいて最適化を実行する目的関数 """
    
    # 1. ブレード枚数
    num_blades = trial.suggest_int("num_blades", 
                                   DESIGN_SPACE['num_blades']['min'], 
                                   DESIGN_SPACE['num_blades']['max'])
    
    # 2. ハブ比率 (YAMLの直径制約から計算)
    min_hub_ratio = MIN_HUB_RADIUS_M / TIP_RADIUS
    max_hub_ratio_limit = (MAX_HUB_DIAMETER_M / 2.0) / TIP_RADIUS
    
    # 矛盾回避
    if max_hub_ratio_limit <= min_hub_ratio:
        max_hub_ratio_limit = min_hub_ratio + 0.01
        
    hub_ratio = trial.suggest_float("hub_ratio", min_hub_ratio, max_hub_ratio_limit)
    
    # 3. ダクト形状
    # YAMLにダクト設定があれば読み込む、なければオープンプロペラ(0.0)とする拡張性
    duct_len = trial.suggest_float("duct_length", 0.0, TIP_RADIUS)
    if duct_len < 1e-6:
        duct_lip = 0.0
    else:
        max_possible_lip = min(MAX_DUCT_LIP_RADIUS_M, duct_len)
        duct_lip = trial.suggest_float("duct_lip_radius", 0.0, max_possible_lip)

    hub_radius = TIP_RADIUS * hub_ratio
    blade_span = TIP_RADIUS - hub_radius
    r_coords_bemt = hub_radius + SPAN_POSITIONS_BEMT * blade_span
    r_coords_airfoil_def = hub_radius + SPAN_POSITIONS_AIRFOIL * blade_span
    
    # 4. 翼型 (3点分布)
    airfoil_names = [
        trial.suggest_categorical("airfoil_0_hub", AIRFOIL_CHOICES),
        trial.suggest_categorical("airfoil_1_mid", AIRFOIL_CHOICES),
        trial.suggest_categorical("airfoil_2_tip", AIRFOIL_CHOICES)
    ]

    # 5. 弦長分布 (YAMLリストから動的生成)
    chord_control_points_y = []
    chord_constraints = DESIGN_SPACE['chord_constraints']
    
    # YAMLの定義数が不足している場合は最後の値を繰り返すなどの安全策をとるか、エラーにする
    if len(chord_constraints) < NUM_GEOM_CONTROL_POINTS:
        raise ValueError(f"Config Error: chord_constraints list length ({len(chord_constraints)}) must match geom_control_points ({NUM_GEOM_CONTROL_POINTS})")

    for i in range(NUM_GEOM_CONTROL_POINTS):
        min_mm, max_mm = chord_constraints[i]
        # mm -> m 変換
        val_m = trial.suggest_float(f"chord_ctrl_{i}", min_mm / 1000.0, max_mm / 1000.0)
        chord_control_points_y.append(val_m)

    # 6. ピッチ角分布 (YAMLリストから動的生成)
    pitch_control_points_y = []
    pitch_constraints = DESIGN_SPACE['pitch_constraints']
    
    if len(pitch_constraints) < NUM_GEOM_CONTROL_POINTS:
        raise ValueError(f"Config Error: pitch_constraints list length must match geom_control_points")

    for i in range(NUM_GEOM_CONTROL_POINTS):
        min_deg, max_deg = pitch_constraints[i]
        val_deg = trial.suggest_float(f"pitch_ctrl_{i}", min_deg, max_deg)
        pitch_control_points_y.append(val_deg)

    # 7. 分布生成
    pitch_distribution = generate_bezier_distribution(pitch_control_points_y, NUM_BEMT_ELEMENTS)
    chord_distribution = generate_bezier_distribution(chord_control_points_y, NUM_BEMT_ELEMENTS)

    # 8. 制約チェック (最小厚み)
    idx_map = np.argmin(np.abs(r_coords_airfoil_def[:, None] - r_coords_bemt), axis=0)
    for i in range(NUM_BEMT_ELEMENTS):
        airfoil_name = airfoil_names[idx_map[i]]
        # 代表Re数での厚みチェック
        _, _, t_c_ratio = get_airfoil_properties(airfoil_name, 10000, 0)
        actual_thickness_m = chord_distribution[i] * t_c_ratio
        
        if actual_thickness_m < MIN_ABSOLUTE_THICKNESS_M: 
            return -9999.0 # 制約違反ペナルティ
            
    # 9. 性能評価 (BEMT)
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

    # 10. 目的関数の計算 (制約付き最大化)
    if P > TARGET_POWER_LIMIT:
        # パワーオーバー時は推力に関わらずペナルティ (目標推力から減算)
        return TARGET_THRUST_MIN - (P - TARGET_POWER_LIMIT) 
    if total_T < TARGET_THRUST_MIN:
        # パワーOKでも推力不足ならそのまま返す
        return total_T 
    
    # 両方クリアなら推力を最大化
    return total_T

# --- 5. メイン実行ブロック ---

if __name__ == "__main__":
    
    output_lines = []
    def log_and_print(message):
        print(message)
        output_lines.append(str(message))
    
    # 探索範囲の計算結果を表示 (確認用)
    actual_min_hub_ratio = MIN_HUB_RADIUS_M / TIP_RADIUS
    actual_max_hub_ratio = (MAX_HUB_DIAMETER_M / 2.0) / TIP_RADIUS
    if actual_max_hub_ratio < actual_min_hub_ratio:
        actual_max_hub_ratio = actual_min_hub_ratio + 0.01

    log_and_print(f"--- 🛠️  Propeller Optimization (Config: {config.get('project', {}).get('name', 'Unknown')}) ---")
    log_and_print(f"Target: Maximize Thrust @ {RPM} RPM")
    log_and_print(f"Constraints: Power <= {TARGET_POWER_LIMIT} W, Thrust >= {TARGET_THRUST_MIN} N")
    log_and_print(f"             Hub Dia: {MIN_HUB_RADIUS_M*2000:.1f}mm - {MAX_HUB_DIAMETER_M*2000:.1f}mm")
    log_and_print(f"             Chord Control Points: {DESIGN_SPACE['chord_constraints']}")
    log_and_print("--------------------------------------------------")

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(direction="maximize")
    
    n_trials = config['optuna']['n_trials']
    n_jobs = config['optuna']['n_jobs']
    
    log_and_print(f"Running Optuna ({n_trials} trials, jobs={n_jobs})...")
    start_time = time.time()
    study.optimize(evaluate_design, n_trials=n_trials, n_jobs=n_jobs)
    end_time = time.time()

    log_and_print(f"\nOptimization finished in {end_time - start_time:.2f} seconds.")
    
    # --- 結果処理 (ベスト解の表示と保存) ---
    if study.best_trial.value < TARGET_THRUST_MIN:
        log_and_print("❌ Optimization FAILED to meet minimum thrust constraint.")
        log_and_print(f"   (Best attempt value: {study.best_trial.value:.4f})")
    else:
        log_and_print("✅ Best solution found:")
        best_trial = study.best_trial
        best_params = best_trial.params
        
        log_and_print(f"  Best Thrust: {best_trial.value:.4f} N")
        log_and_print(f"    num_blades: {best_params['num_blades']}")
        log_and_print(f"    hub_ratio: {best_params['hub_ratio']:.3f} (Dia: {best_params['hub_ratio']*DIAMETER*1000:.1f} mm)")
        
        # 再計算と詳細ログ出力
        # (ロジックは以前と同じですが、config値を使うように注意)
        # 簡略化のため、結果表示ロジックの要点のみ記述します
        
        # ... (これまでのCADデータ出力ロジックをここに配置) ...
        # 注意: chord_constraintsリストを使って再構築する必要があります
        
        # ベストな制御点リストを復元
        best_chord_ctrl = [best_params[f"chord_ctrl_{i}"] for i in range(NUM_GEOM_CONTROL_POINTS)]
        best_pitch_ctrl = [best_params[f"pitch_ctrl_{i}"] for i in range(NUM_GEOM_CONTROL_POINTS)]
        
        chord_dist = generate_bezier_distribution(best_chord_ctrl, NUM_BEMT_ELEMENTS)
        pitch_dist = generate_bezier_distribution(best_pitch_ctrl, NUM_BEMT_ELEMENTS)
        
        # CADデータ表示
        log_and_print("\n--- CAD Data (BEMT Points Definition) ---")
        log_and_print(f"    i | Radius (m) | Pitch (deg) | Chord (mm) | Nearest Airfoil")
        
        # 簡易表示
        hub_rad = TIP_RADIUS * best_params['hub_ratio']
        r_bemt = hub_rad + SPAN_POSITIONS_BEMT * (TIP_RADIUS - hub_rad)
        
        # Propellerオブジェクト作成と翼型判定
        prop_temp = Propeller(
             hub_radius=hub_rad, tip_radius=TIP_RADIUS, num_blades=best_params['num_blades'],
             r_coords=r_bemt, pitch_coords_deg=pitch_dist, chord_coords=chord_dist,
             r_coords_airfoil_def=(hub_rad + SPAN_POSITIONS_AIRFOIL * (TIP_RADIUS - hub_rad)),
             airfoil_names=[best_params["airfoil_0_hub"], best_params["airfoil_1_mid"], best_params["airfoil_2_tip"]],
             duct_length=best_params["duct_length"], duct_lip_radius=best_params["duct_lip_radius"]
        )
        
        airfoil_names_final = [prop_temp.get_airfoil_name(r) for r in r_bemt]

        for i in range(NUM_BEMT_ELEMENTS):
            log_and_print(f"    {i:2d} |   {r_bemt[i]:.4f}   |   {pitch_dist[i]:8.3f} |   {chord_dist[i]*1000:6.1f}   | {airfoil_names_final[i]}")

    # ファイル保存
    output_dir = config['project']['output_dir']
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    timestamp = datetime.now().strftime("%m%d%H%M")
    filename = os.path.join(output_dir, f"result_{timestamp}.txt")
    
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("\n".join(output_lines))
    print(f"\n✅ Results saved to {filename}")

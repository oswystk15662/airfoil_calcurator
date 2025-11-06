# step5_optimize_optuna.py
import numpy as np
import optuna
from bemt_solver.geometry import Propeller
from bemt_solver.core import solve_bemt
import time

# --- 1. 設計の基本パラメータ (Telloのスペック) ---
DIAMETER = 0.076  # 76 mm
TIP_RADIUS = DIAMETER / 2.0
HUB_RATIO = 0.15          
HUB_RADIUS = TIP_RADIUS * HUB_RATIO
NUM_BLADES = 4            
RPM = 15000.0
V_INFINITY = 0.0 # ホバー時の最適化
AIR_DENSITY = 1.225
KINEMATIC_VISCOSITY = 1.4607e-5 # Re計算のため

# --- 2. 最適化の制約 ---
TARGET_POWER_LIMIT = 3.26  # (W) Telloの推定限界パワー
TARGET_THRUST_MIN = 0.196  # (N) 最低でもホバリング推力は確保

# --- 3. 最適化の探索空間 ---
R_COORDS = np.array([
    HUB_RADIUS, 
    TIP_RADIUS * 0.30, 
    TIP_RADIUS * 0.55, 
    TIP_RADIUS * 0.80, 
    TIP_RADIUS
])
# 🔽 [修正] データベースに登録した翼型名 🔽
AIRFOIL_CHOICES = [
    "aquilla",
    "clarky",
    "dae11",
    "dae21",
    "dae31",
    "e61",
    "geminism",
    "goe795",
    "mh32",
    "naca4412",
    "naca6409",
    "s1223",
    "s8035"
]


def evaluate_design(trial):
    """ Optunaが呼び出す目的関数 (5点定義バージョン) """
    
    # --- [修正] 設計変数を 3 -> 5 に増やす ---
    
    # 1. 翼型 (Categorical: 選択肢から選ぶ)
    airfoil_names = [
        trial.suggest_categorical("airfoil_1_hub", AIRFOIL_CHOICES),
        trial.suggest_categorical("airfoil_2_30R", AIRFOIL_CHOICES),
        trial.suggest_categorical("airfoil_3_55R", AIRFOIL_CHOICES),
        trial.suggest_categorical("airfoil_4_80R", AIRFOIL_CHOICES),
        trial.suggest_categorical("airfoil_5_tip", AIRFOIL_CHOICES)
    ]
    
    # 2. 弦長 (Float: 範囲内の少数)
    chord_coords = [
        trial.suggest_float("chord_1_hub", 0.003, 0.005), # 3mm ~ 5mm
        trial.suggest_float("chord_2_30R", 0.003, 0.005),
        trial.suggest_float("chord_3_55R", 0.003, 0.005),
        trial.suggest_float("chord_4_80R", 0.002, 0.005),
        trial.suggest_float("chord_5_tip", 0.002, 0.004) # 先端は細く
    ]

    # 3. ピッチ角 (Float: 範囲内の少数)
    pitch_coords_deg = [
        trial.suggest_float("pitch_1_hub", 15.0, 35.0),
        trial.suggest_float("pitch_2_30R", 12.0, 30.0),
        trial.suggest_float("pitch_3_55R", 10.0, 25.0),
        trial.suggest_float("pitch_4_80R", 5.0, 20.0),
        trial.suggest_float("pitch_5_tip", 5.0, 18.0)
    ]
    # (※ピッチ角は一般的に先端に行くほど浅くなるため、探索範囲を絞っています)
    
    # --- 性能評価 (ここから下は変更なし) ---
    prop = Propeller(
        hub_radius=HUB_RADIUS,
        tip_radius=TIP_RADIUS,
        num_blades=NUM_BLADES,
        r_coords=R_COORDS,
        pitch_coords_deg=np.array(pitch_coords_deg),
        chord_coords=np.array(chord_coords),
        airfoil_names=airfoil_names,
        duct_length=0.0, # ダクトなし (Tello)
        duct_lip_radius=0.0
    )
    
    (total_T, _, _, 
     _, P, _) = solve_bemt(
        prop, 
        v_infinity=V_INFINITY, 
        rpm=RPM, 
        air_density=AIR_DENSITY, 
        kinematic_viscosity=KINEMATIC_VISCOSITY,
        num_elements=10 # 高速化
    )

    # --- 制約条件の判定 (変更なし) ---
    if P > TARGET_POWER_LIMIT:
        return TARGET_THRUST_MIN - (P - TARGET_POWER_LIMIT) 
    if total_T < TARGET_THRUST_MIN:
        return total_T 
    return total_T

# --- 実行 ---
if __name__ == "__main__":
    print("--- 🛠️  Step 4: Advanced Optimization (Optuna + Airfoil DB) ---")
    print(f"Target: Maximize Thrust @ {RPM} RPM (Hover)")
    print(f"Constraints: Power <= {TARGET_POWER_LIMIT} W, Thrust >= {TARGET_THRUST_MIN} N")
    # 🔽 [修正] 5点 x 3種類 = 15変数
    print(f"Optimizing: Pitch (5), Chord (5), Airfoil (5) = 15 variables")
    print("--------------------------------------------------")

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    
    study = optuna.create_study(direction="maximize")
    
    # 🔽 [修正] 変数が増えたため、試行回数を増やす
    n_trials = 300 
    print(f"Running Optuna ({n_trials} trials)...")
    start_time = time.time()
    study.optimize(evaluate_design, n_trials=n_trials)
    end_time = time.time()

    print(f"\nOptimization finished in {end_time - start_time:.2f} seconds.")
    print("--------------------------------------------------")
    
    print("✅ Best solution found:")
    best_trial = study.best_trial
    
    print(f"  Best Thrust: {best_trial.value:.4f} N")
    
    print("\n  Optimal Parameters:")
    for key, value in best_trial.params.items():
        if isinstance(value, float):
            print(f"    {key}: {value:.3f}")
        else:
            print(f"    {key}: {value}")
            
    # --- [修正] 5点の結果を取得するために書き換え ---
    best_params_pitch = [
        best_trial.params["pitch_1_hub"], 
        best_trial.params["pitch_2_30R"],
        best_trial.params["pitch_3_55R"],
        best_trial.params["pitch_4_80R"],
        best_trial.params["pitch_5_tip"]
    ]
    best_params_chord = [
        best_trial.params["chord_1_hub"], 
        best_trial.params["chord_2_30R"],
        best_trial.params["chord_3_55R"],
        best_trial.params["chord_4_80R"],
        best_trial.params["chord_5_tip"]
    ]
    best_params_airfoils = [
        best_trial.params["airfoil_1_hub"], 
        best_trial.params["airfoil_2_30R"],
        best_trial.params["airfoil_3_55R"],
        best_trial.params["airfoil_4_80R"],
        best_trial.params["airfoil_5_tip"]
    ]
    
    prop_final = Propeller(
        hub_radius=HUB_RADIUS, tip_radius=TIP_RADIUS, num_blades=NUM_BLADES,
        r_coords=R_COORDS, pitch_coords_deg=np.array(best_params_pitch),
        chord_coords=np.array(best_params_chord), airfoil_names=best_params_airfoils
    )
    (T_final, _, _, _, P_final, _) = solve_bemt(
        prop_final, V_INFINITY, RPM, AIR_DENSITY, KINEMATIC_VISCOSITY
    )
    
    print("\n  Final Performance Check:")
    print(f"    Thrust: {T_final:.4f} N")
    print(f"    Power:  {P_final:.2f} W (Constraint: <= {TARGET_POWER_LIMIT} W)")
    print(f"    g/W:    {(T_final / 9.81 * 1000) / P_final:.2f}")
    
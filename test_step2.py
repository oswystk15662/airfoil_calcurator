import numpy as np
from bemt_solver.geometry import Propeller
from bemt_solver.core import solve_bemt
import time

def run_bemt_test():
    """
    BEMTソルバー (Step 2) の動作確認テスト
    """
    print("--- 🚀 BEMT Solver Test (Step 2) ---")

    # --- 1. テスト用のプロペラ形状を定義 ---
    # (APC 10x4.7風の簡易モデル)
    
    # 基本諸元
    diameter = 10.0 * 0.0254  # 10インチをメートルに変換
    hub_ratio = 0.15          # ハブ半径/チップ半径 (仮)
    tip_radius = diameter / 2.0
    hub_radius = tip_radius * hub_ratio
    num_blades = 2
    
    # 翼型 (Step 1 で naca2412.dat が配置済みであることを前提)
    airfoil_name = "naca2412"

    # 形状定義点 (半径座標)
    # (ハブ、中間、チップの3点)
    r_coords = np.array([
        hub_radius, 
        tip_radius * 0.7, 
        tip_radius
    ])

    # ピッチ分布 (ねじり下げを再現)
    # (4.7インチピッチ -> 2*pi*r * tan(pitch) = 4.7*0.0254)
    # (ここでは簡易的に、70%半径でピッチ角12度、先端で8度とする)
    pitch_coords_deg = np.array([
        15.0,  # ハブ (度)
        12.0,  # 中間 (度)
        8.0    # チップ (度)
    ])
    
    # コード長 (翼弦長) 分布
    chord_coords = np.array([
        0.030, # ハブ (m)
        0.035, # 中間 (m)
        0.020  # チップ (m)
    ])

    # プロペラオブジェクトの作成
    try:
        prop = Propeller(
            hub_radius=hub_radius,
            tip_radius=tip_radius,
            num_blades=num_blades,
            r_coords=r_coords,
            pitch_coords_deg=pitch_coords_deg,
            chord_coords=chord_coords,
            airfoil_name=airfoil_name
        )
        print(f"Propeller created: {diameter*100/2.54:.1f} inch diameter")
        print(f"Airfoil: {airfoil_name}")
        print("---------------------------------")
    except Exception as e:
        print(f"❌ Error creating Propeller object: {e}")
        return

    # --- 2. 運転条件 ---
    rpm = 5000.0
    air_density = 1.225
    
    # --- ケース1: ホバー (V=0 m/s) ---
    print(f"Running Test Case 1: Hover (V=0 m/s) at {rpm} RPM...")
    print(" (XFOILが複数回呼び出されるため、少し時間がかかります...)")
    start_time = time.time()
    
    try:
        thrust_h, torque_h, power_h, eff_h = solve_bemt(
            prop,
            v_infinity=0.0,
            rpm=rpm,
            air_density=air_density,
            num_elements=20 # 要素分割数
        )
        
        end_time = time.time()
        print(f"  Calculation finished in {end_time - start_time:.2f} seconds.")
        print("  ✅ Hover Test Success!")
        print(f"     Thrust: {thrust_h:.3f} N")
        print(f"     Torque: {torque_h:.3f} Nm")
        print(f"     Power:  {power_h:.2f} W")
        print(f"     Efficiency: {eff_h * 100:.1f} % (Note: 0 by definition in hover)")

    except Exception as e:
        print(f"  ❌ Hover Test Failed: {e}")
        # スタックトレースを表示して詳細なエラー箇所を確認
        import traceback
        traceback.print_exc()

    print("---------------------------------")

    # --- ケース2: 前進飛行 (V=10 m/s) ---
    print(f"Running Test Case 2: Forward Flight (V=10 m/s) at {rpm} RPM...")
    start_time = time.time()
    
    try:
        thrust_f, torque_f, power_f, eff_f = solve_bemt(
            prop,
            v_infinity=10.0,
            rpm=rpm,
            air_density=air_density,
            num_elements=20
        )
        
        end_time = time.time()
        print(f"  Calculation finished in {end_time - start_time:.2f} seconds.")
        print("  ✅ Forward Flight Test Success!")
        print(f"     Thrust: {thrust_f:.3f} N")
        print(f"     Torque: {torque_f:.3f} Nm")
        print(f"     Power:  {power_f:.2f} W")
        print(f"     Efficiency: {eff_f * 100:.1f} %")

    except Exception as e:
        print(f"  ❌ Forward Flight Test Failed: {e}")
        import traceback
        traceback.print_exc()
        
    print("---------------------------------")
    print("Test complete.")

if __name__ == "__main__":
    run_bemt_test()
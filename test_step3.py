import numpy as np
from bemt_solver.geometry import Propeller
from bemt_solver.core import solve_bemt
import time

def run_ducted_bemt_test():
    """
    Ducted BEMTソルバー (Step 3) の動作確認テスト
    ダクトなし(Step2)とダクトあり(Step3)の性能を比較します。
    """
    print("--- 🚀 Ducted BEMT Solver Test (Step 3) ---")

    # --- 1. プロペラ形状の定義 ---
    diameter = 10.0 * 0.0254  # 10インチ
    tip_radius = diameter / 2.0
    hub_ratio = 0.15          # ハブ半径/チップ半径 (仮)
    hub_radius = tip_radius * hub_ratio
    num_blades = 2
    airfoil_name = "naca2412" # xfoil_wrapper/airfoils/ に naca2412.dat が必要

    r_coords = np.array([
        hub_radius, 
        tip_radius * 0.7, 
        tip_radius
    ])

    # 🔽 [修正済] 推力が出るようにピッチ角を大きく設定 🔽
    pitch_coords_deg = np.array([
        25.0,  # ハブ (度)
        22.0,  # 中間 (度)
        18.0   # チップ (度)
    ])
    
    chord_coords = np.array([
        0.030, # ハブ (m)
        0.035, # 中間 (m)
        0.020  # チップ (m)
    ])

    # --- 2. ダクト形状の定義 ---
    duct_length = diameter * 0.5  # d/D = 0.5
    duct_lip_radius = diameter * 0.031
    
    # --- 3. 運転条件 ---
    rpm = 5000.0
    air_density = 1.225
    
    print(f"Propeller Diameter: {diameter:.3f} m ({diameter*100/2.54:.1f} in)")
    print(f"Duct Length (d/D=0.5): {duct_length:.3f} m")
    print(f"Duct Lip Radius: {duct_lip_radius*1000:.1f} mm")
    print("---------------------------------")
    
    # ---
    # テストケース 1: ダクトなし (ベースライン)
    # ---
    print(f"Running Test Case 1: NO DUCT (Baseline)")
    
    prop_baseline = Propeller(
        hub_radius=hub_radius, tip_radius=tip_radius, num_blades=num_blades,
        r_coords=r_coords, pitch_coords_deg=pitch_coords_deg,
        chord_coords=chord_coords, airfoil_name=airfoil_name,
        duct_length=0.0,  # ダクト長ゼロ
        duct_lip_radius=0.0
    )
    
    # --- 1a: ホバー (V=0)
    print(f"  Testing Hover (V=0 m/s) at {rpm} RPM...")
    start_time = time.time()
    try:
        (T_h1, Tf_h1, Td_h1, 
         Q_h1, P_h1, eff_h1) = solve_bemt(prop_baseline, v_infinity=0.0, rpm=rpm)
        
        print(f"  ...Success ({time.time() - start_time:.2f} s)")
        print(f"     Total Thrust:  {T_h1:.3f} N")
        print(f"     (Fan Thrust:   {Tf_h1:.3f} N)")
        print(f"     (Duct Thrust:  {Td_h1:.3f} N)")
        print(f"     Power:         {P_h1:.2f} W")
        
        thrust_h_baseline = T_h1

    except Exception as e:
        print(f"  ❌ Hover Test Failed: {e}")
        import traceback
        traceback.print_exc()
        thrust_h_baseline = 0.0

    # --- 1b: 前進飛行 (V=10)
    print(f"\n  Testing Forward Flight (V=10 m/s) at {rpm} RPM...")
    start_time = time.time()
    try:
        (T_f1, Tf_f1, Td_f1, 
         Q_f1, P_f1, eff_f1) = solve_bemt(prop_baseline, v_infinity=10.0, rpm=rpm)
        
        print(f"  ...Success ({time.time() - start_time:.2f} s)")
        print(f"     Total Thrust:  {T_f1:.3f} N")
        print(f"     (Fan Thrust:   {Tf_f1:.3f} N)")
        print(f"     (Duct Thrust:  {Td_f1:.3f} N)")
        print(f"     Power:         {P_f1:.2f} W")
        print(f"     Efficiency:    {eff_f1 * 100:.1f} %")
    except Exception as e:
        print(f"  ❌ Forward Flight Test Failed: {e}")
        import traceback
        traceback.print_exc()

    print("=================================")

    # ---
    # テストケース 2: ダクトあり
    # ---
    print(f"Running Test Case 2: WITH DUCT")
    
    prop_ducted = Propeller(
        hub_radius=hub_radius, tip_radius=tip_radius, num_blades=num_blades,
        r_coords=r_coords, pitch_coords_deg=pitch_coords_deg,
        chord_coords=chord_coords, airfoil_name=airfoil_name,
        duct_length=duct_length,
        duct_lip_radius=duct_lip_radius
    )
    
    # --- 2a: ホバー (V=0)
    print(f"  Testing Hover (V=0 m/s) at {rpm} RPM...")
    start_time = time.time()
    try:
        (T_h2, Tf_h2, Td_h2, 
         Q_h2, P_h2, eff_h2) = solve_bemt(prop_ducted, v_infinity=0.0, rpm=rpm)
        
        print(f"  ...Success ({time.time() - start_time:.2f} s)")
        print(f"     Total Thrust:  {T_h2:.3f} N  <-- 注目")
        print(f"     (Fan Thrust:   {Tf_h2:.3f} N)")
        print(f"     (Duct Thrust:  {Td_h2:.3f} N)  <-- 注目")
        print(f"     Power:         {P_h2:.2f} W")
        
        thrust_h_ducted = T_h2

    except Exception as e:
        print(f"  ❌ Hover Test Failed: {e}")
        import traceback
        traceback.print_exc()
        thrust_h_ducted = 0.0

    # --- 2b: 前進飛行 (V=10)
    print(f"\n  Testing Forward Flight (V=10 m/s) at {rpm} RPM...")
    start_time = time.time()
    try:
        (T_f2, Tf_f2, Td_f2, 
         Q_f2, P_f2, eff_f2) = solve_bemt(prop_ducted, v_infinity=10.0, rpm=rpm)
        
        print(f"  ...Success ({time.time() - start_time:.2f} s)")
        print(f"     Total Thrust:  {T_f2:.3f} N")
        print(f"     (Fan Thrust:   {Tf_f2:.3f} N)")
        print(f"     (Duct Thrust:  {Td_f2:.3f} N)")
        print(f"     Power:         {P_f2:.2f} W")
        print(f"     Efficiency:    {eff_f2 * 100:.1f} %")
    except Exception as e:
        print(f"  ❌ Forward Flight Test Failed: {e}")
        import traceback
        traceback.print_exc()
        
    print("---------------------------------")
    
    # --- 4. サマリー ---
    print("--- 📊 Test Summary (Hover Thrust) ---")
    print(f"Baseline (No Duct): {thrust_h_baseline:.3f} N")
    print(f"With Ducted Model:  {thrust_h_ducted:.3f} N")
    
    if thrust_h_baseline > 0.001:
        increase = ((thrust_h_ducted - thrust_h_baseline) / thrust_h_baseline) * 100
        print(f"Thrust Increase: +{increase:.1f}%")
    
    print("Test complete.")

if __name__ == "__main__":
    run_ducted_bemt_test()

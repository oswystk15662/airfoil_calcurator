# test_step4_tello.py
import numpy as np
from bemt_solver.geometry import Propeller
from bemt_solver.core import solve_bemt
import time

def run_tello_test():
    """
    DJI Tello のパラメータでBEMTソルバーをテストする (低Re対応)
    """
    print("--- 🚀 Tello Propeller Analysis (Step 3/4) ---")

    # --- 1. Telloのプロペラ形状を定義 ---
    diameter = 0.076  # 76 mm
    tip_radius = diameter / 2.0
    hub_ratio = 0.04
    hub_radius = tip_radius * hub_ratio
    num_blades = 4  # 4枚ブレード
    
    # 翼型名はダミー (XFOILを使わないため)
    airfoil_name = "low_re_model" 

    # 形状定義点 (半径座標)
    r_coords = np.array([
        hub_radius,      # ハブ
        tip_radius * 0.7,  # 中間
        tip_radius       # チップ
    ])

    # ピッチ分布 (ねじり下げ)
    # ※これは推力が 0.196N になるように調整する必要がある「設計変数」
    #   ひとまず、妥当と思われる値からスタート
    pitch_coords_deg = np.array([
        30.0,  # ハブ (度)
        25.0,  # 中間 (度)
        20.0   # チップ (度)
    ])
    
    # コード長 (翼弦長) 分布 (最大5mm)
    chord_coords = np.array([
        0.004, # ハブ (m)
        0.005, # 中間 (m)
        0.003  # チップ (m)
    ])

    # --- 2. ダクト形状の定義 ---
    # Telloの純正プロペラはダクトなし (プロペラガードは別)
    # ダクトなし(Baseline)と、仮想のダクトありを比較
    duct_length_virtual = diameter * 0.5  # d/D = 0.5
    duct_lip_radius_virtual = diameter * 0.031

    # --- 3. 運転条件 (Telloホバー時) ---
    rpm = 15000.0
    v_infinity = 0.0 # ホバー
    air_density = 1.225

    # --- 4. 空力モデルのパラメータ (低Re用) ---
    # OptDuct論文 (8章) に基づく
    aero_params = {
        "lift_slope_rad": 2 * np.pi * 0.9,  # 揚力傾斜 (rad-1), 3D効果で 2pi より少し小さい
        "zero_lift_aoa_deg": -2.0,          # ゼロ揚力角 (deg), キャンバー翼型を想定
        "cd_profile": 0.02                  # 形状抗力係数 (低Reなので高め)
    }
    
    print(f"Propeller: Tello (D={diameter*1000:.0f}mm, B={num_blades})")
    print(f"Operating at: {rpm:.0f} RPM, {v_infinity:.1f} m/s (Hover)")
    print(f"Target Thrust: 0.196 N")
    print(f"Estimated Power: ~3.1 W")
    print("---------------------------------")
    
    # ---
    # ケース1: ダクトなし (Tello純正状態)
    # ---
    print(f"Running Test Case 1: NO DUCT (Baseline Tello)")
    
    prop_baseline = Propeller(
        hub_radius=hub_radius, tip_radius=tip_radius, num_blades=num_blades,
        r_coords=r_coords, pitch_coords_deg=pitch_coords_deg,
        chord_coords=chord_coords, airfoil_name=airfoil_name,
        duct_length=0.0, duct_lip_radius=0.0
    )
    
    start_time = time.time()
    try:
        (T_h1, Tf_h1, Td_h1, 
         Q_h1, P_h1, eff_h1) = solve_bemt(prop_baseline, v_infinity, rpm, air_density, num_elements=20, **aero_params)
        
        print(f"  ...Success ({time.time() - start_time:.2f} s)")
        print(f"     Total Thrust:  {T_h1:.3f} N")
        print(f"     Power:         {P_h1:.2f} W")

    except Exception as e:
        print(f"  ❌ Hover Test Failed: {e}")
        import traceback
        traceback.print_exc()

    print("=================================")

    # ---
    # ケース2: 仮想ダクトあり
    # ---
    print(f"Running Test Case 2: WITH VIRTUAL DUCT")
    
    prop_ducted = Propeller(
        hub_radius=hub_radius, tip_radius=tip_radius, num_blades=num_blades,
        r_coords=r_coords, pitch_coords_deg=pitch_coords_deg,
        chord_coords=chord_coords, airfoil_name=airfoil_name,
        duct_length=duct_length_virtual,
        duct_lip_radius=duct_lip_radius_virtual
    )
    
    start_time = time.time()
    try:
        (T_h2, Tf_h2, Td_h2, 
         Q_h2, P_h2, eff_h2) = solve_bemt(prop_ducted, v_infinity, rpm, air_density, num_elements=20, **aero_params)
        
        print(f"  ...Success ({time.time() - start_time:.2f} s)")
        print(f"     Total Thrust:  {T_h2:.3f} N")
        print(f"     (Fan Thrust:   {Tf_h2:.3f} N)")
        print(f"     (Duct Thrust:  {Td_h2:.3f} N)")
        print(f"     Power:         {P_h2:.2f} W")

    except Exception as e:
        print(f"  ❌ Hover Test Failed: {e}")
        import traceback
        traceback.print_exc()
        
    print("---------------------------------")
    print("Test complete.")

if __name__ == "__main__":
    run_tello_test()

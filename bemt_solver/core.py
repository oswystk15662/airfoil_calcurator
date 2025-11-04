# bemt_solver/core.py
import numpy as np
from scipy.optimize import fsolve
from .geometry import Propeller
from .losses import prandtl_loss_factor
from .duct import calculate_wake_contraction
from xfoil_wrapper.core import get_airfoil_performance

def solve_bemt(prop: Propeller, v_infinity: float, rpm: float,
               air_density: float = 1.225,
               kinematic_viscosity: float = 1.4607e-5,
               num_elements: int = 20):
    """
    BEMT(翼素運動量理論)ソルバーのメイン関数。
    (ホバー/前進飛行 対応の一般化モデル)

    Args:
        prop (Propeller): プロペラ形状オブジェクト
        v_infinity (float): 対気速度 (m/s)
        rpm (float): 回転数 (RPM)
        air_density (float): 空気密度 (kg/m^3)
        kinematic_viscosity (float): 動粘性係数 (m^2/s)
        num_elements (int): ブレードの分割数

    Returns:
        (total_thrust, total_fan_thrust, total_duct_thrust, total_torque, power, efficiency)
    """
    
    omega = rpm * 2.0 * np.pi / 60.0 # RPMをrad/sに変換
    
    # ダクト効果の計算 (OptDuct理論より)
    k_squared = calculate_wake_contraction(prop)
    fan_thrust_fraction = 0.5 * k_squared 
    lip_factor = 1.0 - fan_thrust_fraction
    
    # ブレードをハブからチップまで分割
    radii = np.linspace(prop.hub_radius, prop.tip_radius, num_elements + 1)
    r_centers = (radii[:-1] + radii[1:]) / 2.0 # 各要素の中心半径
    dr = radii[1] - radii[0] # 各要素の幅
    
    total_fan_thrust_elements = 0.0 # ブレード推力の合計
    total_torque_elements = 0.0     # ブレードトルクの合計
    
    # --- 各ブレード要素について計算 ---
    for r in r_centers:
        
        # --- 1. 要素の形状を取得 ---
        chord = prop.get_chord(r)
        pitch_rad = np.radians(prop.get_pitch_deg(r))
        sigma = (prop.num_blades * chord) / (2.0 * np.pi * r)
        
        # --- 2. 誘導速度 (v_i, a_prime) を解く ---
        # fsolve を使い、残差(residual)が0になる v_i と a_prime を見つける
        # x[0] = v_i (軸方向誘導速度 [m/s])
        # x[1] = a_prime (接線方向誘導係数)
        
        def residuals(x):
            v_i = x[0]
            a_prime = x[1]
            
            # --- フロー計算 ---
            v_axial = v_infinity + v_i      # 軸方向の全速度
            v_tangential = omega * r * (1.0 - a_prime) # 接線方向の全速度
            
            phi_rad = np.arctan2(v_axial, v_tangential)
            W_sq = v_axial**2 + v_tangential**2 # 相対速度の2乗
            
            if W_sq < 1e-4:
                return (1.0, 1.0) # 速度がゼロ = 物理的に無効

            # --- 翼型性能 ---
            aoa_rad = pitch_rad - phi_rad
            aoa_deg = np.degrees(aoa_rad)
            
            reynolds = (np.sqrt(W_sq) * chord) / kinematic_viscosity
            cl, cd, _ = get_airfoil_performance(prop.airfoil_name, reynolds, aoa_deg)
            
            if cl is None or cd is None:
                # XFOILが収束しなかった (失速など)
                return (1.0, 1.0) # fsolveに「この推測値は無効」と伝える

            # --- 翼素理論 (Blade Element) ---
            # 流れの座標系 (x:軸方向, y:接線方向)
            C_x = cl * np.cos(phi_rad) - cd * np.sin(phi_rad)
            C_y = cl * np.sin(phi_rad) + cd * np.cos(phi_rad)
            
            # 翼素が発生する推力とトルク (単位スパンあたり)
            dT_blade_dr = 0.5 * air_density * W_sq * (prop.num_blades * chord) * C_x
            dQ_blade_dr = 0.5 * air_density * W_sq * (prop.num_blades * chord) * C_y * r

            # --- 運動量理論 (Momentum Theory) ---
            F = prandtl_loss_factor(r, prop.hub_radius, prop.tip_radius, prop.num_blades, phi_rad)
            
            # 運動量理論による全推力 (ファン+ダクト) (単位スパンあたり)
            # T_total = 2 * rho * Area * (V_inf + v_i) * v_i
            # dArea = 2 * pi * r * dr
            # dT_total_mom_dr = 4.0 * np.pi * r * air_density * v_axial * v_i * F
            
            # Glauert/Buhl 補正 (高推力時)
            # v_i / V_inf に相当する 'a' を計算して判定
            a_eff = v_i / v_infinity if v_infinity > 0.1 else 100.0 # ホバー時は常に高推力状態
            a_threshold = 0.35
            
            if a_eff > a_threshold: 
                # Glauertの補正（Buhlの修正）
                # C_T (局所推力係数) = dT_blade_dr / (0.5 * air_density * (np.pi * r**2) * v_axial**2) ... 複雑
                # OptDuctの(4.6-3)式に至るアプローチはVLM(渦法)ベースであり、
                # BEMTのGlauert補正とは直接互換性がない。
                # ここでは、簡略化された一般運動量式 dT = 4 * pi * r * rho * F * v_i * (V_inf + v_i) を使う。
                # ただし、v_i が V_inf を大きく超えるホバーに近い領域では、
                # dT = 4 * pi * r * rho * F * v_i^2 (ホバーの運動量式)
                
                # 軸方向速度 (v_axial = V_inf + v_i) を使った一般化運動量式
                dT_total_mom_dr = 4.0 * np.pi * r * air_density * v_axial * v_i * F
            else:
                # 通常の前進飛行
                dT_total_mom_dr = 4.0 * np.pi * r * air_density * v_axial * v_i * F

            # OptDuctの理論に基づき、ファン推力に換算
            dT_fan_mom_dr = dT_total_mom_dr * fan_thrust_fraction
            
            # 運動量理論によるトルク (単位スパンあたり)
            # dQ = 4 * pi * r^2 * rho * (V_inf + v_i) * v_t * F
            # v_t = a_prime * omega * r
            v_t = a_prime * omega * r
            dQ_mom_dr = 4.0 * np.pi * r**2 * air_density * v_axial * v_t * F
            
            # --- 残差 ---
            # (翼素理論の力) - (運動量理論の力) = 0 を目指す
            res_thrust = dT_blade_dr - dT_fan_mom_dr
            res_torque = dQ_blade_dr - dQ_mom_dr
            
            return (res_thrust, res_torque)
        
        # --- fsolveで [v_i, a_prime] を解く ---
        try:
            # 初期推測値 (ホバーでも機能するように v_i=1.0 m/s から開始)
            v_i_init = 1.0
            a_prime_init = 0.01 
            
            (v_i_solved, a_prime_solved), _, ier, _ = fsolve(
                residuals, [v_i_init, a_prime_init], xtol=1e-5, maxfev=100, full_output=True
            )
            
            if ier != 1: # 収束しなかった場合
                # print(f"Warning: fsolve did not converge at r={r:.3f}.")
                v_i_solved, a_prime_solved = 0.0, 0.0

        except Exception as e:
            # print(f"Warning: fsolve failed at r={r:.3f}. {e}") # デバッグ用
            v_i_solved, a_prime_solved = 0.0, 0.0

        # --- 3. 収束した値で最終的な推力・トルクを計算 ---
        v_i_final = v_i_solved
        a_prime_final = a_prime_solved
        
        v_axial_final = v_infinity + v_i_final
        v_tan_final = omega * r * (1.0 - a_prime_final)
        phi_final = np.arctan2(v_axial_final, v_tan_final)
        W_sq_final = v_axial_final**2 + v_tan_final**2
        
        if W_sq_final < 1e-6:
            dT, dQ = 0.0, 0.0
        else:
            aoa_deg_final = np.degrees(pitch_rad - phi_final)
            reynolds_final = (np.sqrt(W_sq_final) * chord) / kinematic_viscosity
            cl_final, cd_final, _ = get_airfoil_performance(prop.airfoil_name, reynolds_final, aoa_deg_final)
            
            if cl_final is None: 
                cl_final, cd_final = 0.0, 0.2
                
            C_x_final = cl_final * np.cos(phi_final) - cd_final * np.sin(phi_final)
            C_y_final = cl_final * np.sin(phi_final) + cd_final * np.cos(phi_final)

            # 1要素あたりの推力 (dT) とトルク (dQ)
            dT = 0.5 * air_density * W_sq_final * (prop.num_blades * chord) * C_x_final * dr
            dQ = 0.5 * air_density * W_sq_final * (prop.num_blades * chord) * C_y_final * r * dr
        
        total_fan_thrust_elements += dT
        total_torque_elements += dQ
        
    # --- 4. 最終的な性能を計算 ---
    total_fan_thrust = total_fan_thrust_elements
    total_torque = total_torque_elements
    power = total_torque * omega
    
    # 全推力とダクト推力を計算
    if fan_thrust_fraction > 1e-6:
        total_thrust = total_fan_thrust / fan_thrust_fraction
        # ホバー(V_inf=0)の時、ダクト推力はダクトなし(k_squared=2, fraction=1)で0
        # ダクトあり(k_squared < 2)で total_thrust > total_fan_thrust となる
    else:
        total_thrust = total_fan_thrust

    total_duct_thrust = total_thrust - total_fan_thrust

    if power > 1e-6 and v_infinity > 0.01:
        # 効率は「全推力」で計算する
        efficiency = (total_thrust * v_infinity) / power
    else:
        efficiency = 0.0
        
    return total_thrust, total_fan_thrust, total_duct_thrust, total_torque, power, efficiency

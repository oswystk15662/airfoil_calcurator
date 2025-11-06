# bemt_solver/core.py
import numpy as np
from scipy.optimize import fsolve
from .geometry import Propeller
from .losses import prandtl_loss_factor
from .duct import calculate_wake_contraction
# XFOILラッパーは不要なので削除

def solve_bemt(prop: Propeller, v_infinity: float, rpm: float,
               air_density: float = 1.225,
               # --- [変更] XFOILの代わりに簡易空力モデルのパラメータを追加 ---
               lift_slope_rad: float = 2 * np.pi * 0.9, # 揚力傾斜 (rad-1) ※2D理論値x0.9
               zero_lift_aoa_deg: float = -2.0,      # ゼロ揚力角 (deg)
               cd_profile: float = 0.02,              # 形状抗力係数 (Cd0)
               # ----------------------------------------------------
               num_elements: int = 20
               ):
    """
    BEMT(翼素運動量理論)ソルバーのメイン関数。
    (低Re対応。XFOILの代わりに簡易空力モデルを使用)

    Args:
        (略)
        lift_slope_rad (float): 揚力傾斜 (1/rad)
        zero_lift_aoa_deg (float): ゼロ揚力角 (度)
        cd_profile (float): 形状抗力係数 (Cd0)

    Returns:
        (total_thrust, total_fan_thrust, total_duct_thrust, total_torque, power, efficiency)
    """
    
    omega = rpm * 2.0 * np.pi / 60.0 # RPMをrad/sに変換
    
    # ゼロ揚力角をラジアンに変換
    zero_lift_aoa_rad = np.radians(zero_lift_aoa_deg)
    
    # ダクト効果の計算
    k_squared = calculate_wake_contraction(prop)
    fan_thrust_fraction = 0.5 * k_squared 
    lip_factor = 1.0 - fan_thrust_fraction
    
    radii = np.linspace(prop.hub_radius, prop.tip_radius, num_elements + 1)
    r_centers = (radii[:-1] + radii[1:]) / 2.0
    dr = radii[1] - radii[0] 
    
    total_fan_thrust_elements = 0.0
    total_torque_elements = 0.0
    
    for r in r_centers:
        chord = prop.get_chord(r)
        pitch_rad = np.radians(prop.get_pitch_deg(r))
        sigma = (prop.num_blades * chord) / (2.0 * np.pi * r)
        
        def residuals(x):
            v_i = x[0]
            a_prime = x[1]
            
            v_axial = v_infinity + v_i
            v_tangential = omega * r * (1.0 - a_prime)
            
            phi_rad = np.arctan2(v_axial, v_tangential)
            W_sq = v_axial**2 + v_tangential**2
            
            if W_sq < 1e-4:
                return (1.0, 1.0) 

            # --- [変更] XFOILの代わりに空力モデルを使用 ---
            aoa_rad = pitch_rad - phi_rad
            
            # 簡易揚力モデル
            cl = lift_slope_rad * (aoa_rad - zero_lift_aoa_rad)
            # 簡易抗力モデル (形状抗力のみ)
            cd = cd_profile
            # --- [変更ここまで] ---
            
            C_x = cl * np.cos(phi_rad) - cd * np.sin(phi_rad)
            C_y = cl * np.sin(phi_rad) + cd * np.cos(phi_rad)
            
            dT_blade_dr = 0.5 * air_density * W_sq * (prop.num_blades * chord) * C_x
            dQ_blade_dr = 0.5 * air_density * W_sq * (prop.num_blades * chord) * C_y * r

            F = prandtl_loss_factor(r, prop.hub_radius, prop.tip_radius, prop.num_blades, phi_rad)
            
            a = v_i / v_infinity if v_infinity > 0.1 else 100.0
            a_threshold = 0.35
            
            if a > a_threshold: 
                if v_infinity < 0.1: # ホバー
                     dT_total_mom_dr = 4.0 * np.pi * r * air_density * v_i**2 * F
                else: # 前進飛行 (高推力)
                     dT_total_mom_dr = 4.0 * np.pi * r * air_density * v_axial * v_i * F
            else: # 前進飛行 (通常)
                dT_total_mom_dr = 4.0 * np.pi * r * air_density * v_axial * v_i * F

            dT_fan_mom_dr = dT_total_mom_dr * fan_thrust_fraction
            
            v_t = a_prime * omega * r
            dQ_mom_dr = 4.0 * np.pi * r**2 * air_density * v_axial * v_t * F
            
            res_thrust = dT_blade_dr - dT_fan_mom_dr
            res_torque = dQ_blade_dr - dQ_mom_dr
            
            return (res_thrust, res_torque)
        
        try:
            v_i_init = 5.0 # 初期誘導速度 (m/s)
            a_prime_init = 0.01 
            
            (v_i_solved, a_prime_solved), _, ier, _ = fsolve(
                residuals, [v_i_init, a_prime_init], xtol=1e-5, maxfev=100, full_output=True
            )
            
            if ier != 1: v_i_solved, a_prime_solved = 0.0, 0.0
        except Exception:
            v_i_solved, a_prime_solved = 0.0, 0.0

        # --- 3. 収束した値で最終的な推力・トルクを計算 ---
        v_i_final = v_i_solved
        a_prime_final = np.clip(a_prime_solved, -1.0, 1.0) 
        
        v_axial_final = v_infinity + v_i_final
        v_tan_final = omega * r * (1.0 - a_prime_final)
        phi_final = np.arctan2(v_axial_final, v_tan_final)
        W_sq_final = v_axial_final**2 + v_tan_final**2
        
        if W_sq_final < 1e-6:
            dT, dQ = 0.0, 0.0
        else:
            # --- [変更] XFOILの代わりに空力モデルを使用 ---
            aoa_rad_final = pitch_rad - phi_final
            cl_final = lift_slope_rad * (aoa_rad_final - zero_lift_aoa_rad)
            cd_final = cd_profile
            # --- [変更ここまで] ---
                
            C_x_final = cl_final * np.cos(phi_final) - cd_final * np.sin(phi_final)
            C_y_final = cl_final * np.sin(phi_final) + cd_final * np.cos(phi_final)

            dT = 0.5 * air_density * W_sq_final * (prop.num_blades * chord) * C_x_final * dr
            dQ = 0.5 * air_density * W_sq_final * (prop.num_blades * chord) * C_y_final * r * dr
        
        total_fan_thrust_elements += dT
        total_torque_elements += dQ
        
    total_fan_thrust = total_fan_thrust_elements
    total_torque = total_torque_elements
    power = total_torque * omega
    
    if fan_thrust_fraction > 1e-6 and abs(total_fan_thrust) > 1e-6:
        total_thrust = total_fan_thrust / fan_thrust_fraction
    else:
        total_thrust = total_fan_thrust

    total_duct_thrust = total_thrust - total_fan_thrust

    if power > 1e-6 and v_infinity > 0.01:
        efficiency = (total_thrust * v_infinity) / power
    else:
        efficiency = 0.0
        
    return total_thrust, total_fan_thrust, total_duct_thrust, total_torque, power, efficiency

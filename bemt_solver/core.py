# bemt_solver/core.py
import numpy as np
from scipy.optimize import fsolve
from .geometry import Propeller
from .losses import prandtl_loss_factor
from .duct import calculate_wake_contraction
# 🔽 [修正] 新しいデータベースファイルからインポート 🔽
from airfoil_database_airfoiltools import get_airfoil_performance 

# 実行が成功したことを確認するため、デバッグプリントを V4 に変更
print("--- [DEBUG] Loading BEMT Solver (Database-driven, V4-AirfoilTools) ---")

def solve_bemt(prop: Propeller, v_infinity: float, rpm: float,
               air_density: float = 1.225,
               kinematic_viscosity: float = 1.4607e-5, # Re計算のため
               num_elements: int = 20
               ):
    """
    BEMT(翼素運動量理論)ソルバーのメイン関数。
    (airfoil_database を参照するモデル)
    """
    
    omega = rpm * 2.0 * np.pi / 60.0 
    
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
        
        # この要素で使用する翼型名を取得
        airfoil_name = prop.get_airfoil_name(r)
        
        def residuals(x):
            v_i = x[0]
            a_prime = x[1]
            
            v_axial = v_infinity + v_i
            v_tangential = omega * r * (1.0 - a_prime)
            phi_rad = np.arctan2(v_axial, v_tangential)
            W_sq = v_axial**2 + v_tangential**2
            
            if W_sq < 1e-4: return (1.0, 1.0) 

            aoa_rad = pitch_rad - phi_rad
            aoa_deg = np.degrees(aoa_rad)
            
            # データベースを呼び出す
            reynolds = (np.sqrt(W_sq) * chord) / kinematic_viscosity
            cl, cd, _ = get_airfoil_performance(airfoil_name, reynolds, aoa_deg)
            
            C_x = cl * np.cos(phi_rad) - cd * np.sin(phi_rad)
            C_y = cl * np.sin(phi_rad) + cd * np.cos(phi_rad)
            
            dT_blade_dr = 0.5 * air_density * W_sq * (prop.num_blades * chord) * C_x
            dQ_blade_dr = 0.5 * air_density * W_sq * (prop.num_blades * chord) * C_y * r

            F = prandtl_loss_factor(r, prop.hub_radius, prop.tip_radius, prop.num_blades, phi_rad)
            
            a = v_i / v_infinity if v_infinity > 0.1 else 100.0
            a_threshold = 0.35
            
            dT_total_mom_dr = 0.0
            if a > a_threshold: 
                if v_infinity < 0.1: # ホバー
                     # T = 2 * rho * dA * v_i^2 = 2 * rho * (2*pi*r*dr) * v_i^2
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
            v_i_init = 5.0
            a_prime_init = 0.01 
            (v_i_solved, a_prime_solved), _, ier, _ = fsolve(
                residuals, [v_i_init, a_prime_init], xtol=1e-5, maxfev=100, full_output=True
            )
            if ier != 1: v_i_solved, a_prime_solved = 0.0, 0.0
        except Exception:
            v_i_solved, a_prime_solved = 0.0, 0.0

        v_i_final = v_i_solved
        a_prime_final = np.clip(a_prime_solved, -1.0, 1.0) 
        v_axial_final = v_infinity + v_i_final
        v_tan_final = omega * r * (1.0 - a_prime_final)
        phi_final = np.arctan2(v_axial_final, v_tan_final)
        W_sq_final = v_axial_final**2 + v_tan_final**2
        
        if W_sq_final < 1e-6:
            dT, dQ = 0.0, 0.0
        else:
            aoa_rad_final = pitch_rad - phi_final
            aoa_deg_final = np.degrees(aoa_rad_final)
            reynolds_final = (np.sqrt(W_sq_final) * chord) / kinematic_viscosity
            
            cl_final, cd_final, _ = get_airfoil_performance(airfoil_name, reynolds_final, aoa_deg_final)
            
            C_x_final = cl_final * np.cos(phi_final) - cd_final * np.sin(phi_final) 
            C_y_final = cl_final * np.sin(phi_final) + cd_final * np.cos(phi_final)

            dT = 0.5 * air_density * W_sq_final * (prop.num_blades * chord) * C_x_final * dr
            dQ = 0.5 * air_density * W_sq_final * (prop.num_blades * chord) * C_y_final * r * dr
        
        total_fan_thrust_elements += dT
        total_torque_elements += dQ
        
    # --- 4. 最終的な性能を計算 ---
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

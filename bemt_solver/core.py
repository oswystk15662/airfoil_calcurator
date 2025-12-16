import numpy as np
from scipy.optimize import fsolve
from . import losses
from . import duct
# 🔽 [修正] 新しい関数名をインポート
from airfoil_database_airfoiltools import get_airfoil_properties

def solve_bemt(prop, v_infinity, rpm, air_density, kinematic_viscosity, num_elements=20):
    """
    BEMTを用いてプロペラ（およびダクト）の推力とトルクを計算する。
    [修正] get_airfoil_properties に対応。
    """
    
    # --- 1. ジオメトリの準備 ---
    R = prop.tip_radius
    R_hub = prop.hub_radius
    B = prop.num_blades
    omega = rpm * 2 * np.pi / 60.0
    
    # ブレードを要素に分割 (ハブからチップまで)
    r_elements = np.linspace(R_hub, R, num_elements + 1)
    r_mid = (r_elements[:-1] + r_elements[1:]) / 2.0  # 各要素の中心半径
    dr = r_elements[1] - r_elements[0]               # 要素の幅

    total_thrust_fan = 0.0
    total_torque = 0.0
    
    # ダクトの影響 (OptDuctモデル)
    # k^2 = S_fan / S_wake (後流収縮比)
    k_squared = duct.calculate_wake_contraction(prop)
    
    # リップ推力係数 F_lip (ダクトが推力を分担する割合)
    # T_total = T_fan / F_lip
    # F_lip = 1.0 - 0.5 * k^2  (OptDuct Eq 5.2-12)
    # ダクトなしなら k^2=2.0 -> F_lip=0.0 となり発散するため、
    # 物理的な意味合いから、ダクトなし(k^2=2)の場合は F_lip=1.0 (全推力がファン) とするロジックが必要。
    
    if prop.duct_length <= 0.0:
        F_lip = 1.0 # ダクトなし
    else:
        # OptDuct理論値 (k^2 < 2.0 のはず)
        F_lip = 1.0 - 0.5 * k_squared
        # 安全策: F_lipが0以下にならないようにクリップ (通常ありえないが)
        F_lip = max(F_lip, 0.01)

    # --- 2. 各要素での計算 ---
    for r in r_mid:
        # 幾何形状の取得
        chord = prop.get_chord(r)
        pitch_deg = prop.get_pitch_deg(r)
        airfoil_name = prop.get_airfoil_name(r)
        beta = np.radians(pitch_deg) # ピッチ角 (rad)
        
        sigma = (B * chord) / (2 * np.pi * r) # ソリディティ
        
        # 局所速度 (回転成分)
        V_rot = omega * r
        
        # --- 誘導速度の収束計算 (fsolve) ---
        # 変数: phi (流入角)
        
        def residuals(phi_guess):
            phi = float(phi_guess)
            if phi <= 0 or phi >= np.pi/2:
                return 1.0 # エラー回避
            
            # 局所迎角
            alpha = beta - phi
            aoa_deg = np.degrees(alpha)
            
            # 合成速度
            W = V_rot / np.cos(phi)
            W_sq = W**2
            
            # レイノルズ数
            reynolds = (W * chord) / kinematic_viscosity
            
            # 🔽 [修正] 3つの戻り値を受け取り、3つ目(t/c)は捨てる
            cl, cd, _ = get_airfoil_properties(airfoil_name, reynolds, aoa_deg)
            # 🔼 [修正]
            
            # ブレード要素の力係数 (回転面座標系)
            C_x = cl * np.cos(phi) - cd * np.sin(phi) # 推力方向
            # C_y = cl * np.sin(phi) + cd * np.cos(phi) # 回転抵抗方向
            
            # プラントルの損失係数 F (先端 + ハブ)
            F = losses.prandtl_tip_loss(B, r, R, phi) * losses.prandtl_hub_loss(B, r, R_hub, phi)
            F = max(F, 1e-4) # ゼロ除算回避
            
            # 運動量理論とのバランス式 (fsolveでゼロになるphiを探す)
            # (sigma * C_x) / (4 * F * sin(phi)^2)  =  (v_axial / V_tip) ... の変形
            
            # ここでは簡易的に BEMTの基本式:
            # sin(phi) = v_axial_local / W  <-- 未知数が絡むので
            # 典型的な繰り返し式:
            #   4 * F * sin(phi) * tan(phi) = sigma * Cl * ... 
            # よりも、推力係数の一致を見る形式が安定しやすい。
            
            # 今回は「流入角 phi」を探索するシンプルな形式を採用
            # v_axial = V_infinity + v_induced
            # tan(phi) = v_axial / V_rot
            
            lhs = 4 * F * np.sin(phi) * np.tan(phi)
            rhs = sigma * C_x # 近似: Cl >> Cd なので C_x ≒ Cl * cos(phi)
            
            # V_infがある場合、もう少し複雑になるが、Hover (V=0) ならこれでOK
            return lhs - rhs

        # 初期推定値
        phi_init = np.arctan2(0.1 * V_rot, V_rot) # 適当な初期値
        
        phi_solution = fsolve(residuals, phi_init)
        phi_final = float(phi_solution[0])
        
        # --- 3. 収束後の値で力を計算 ---
        W_final = V_rot / np.cos(phi_final)
        alpha_final = beta - phi_final
        reynolds_final = (W_final * chord) / kinematic_viscosity
        
        # 🔽 [修正] 3つの戻り値を受け取る
        cl_final, cd_final, _ = get_airfoil_properties(airfoil_name, reynolds_final, np.degrees(alpha_final))
        # 🔼 [修正]
        
        # 力の係数
        C_x_final = cl_final * np.cos(phi_final) - cd_final * np.sin(phi_final)
        C_y_final = cl_final * np.sin(phi_final) + cd_final * np.cos(phi_final)
        
        # 要素の推力とトルク
        # dL = 0.5 * rho * W^2 * chord * cl * dr
        # dT = B * (dL * cos(phi) - dD * sin(phi))
        #    = 0.5 * rho * W^2 * B * chord * C_x * dr
        
        dT_elem = 0.5 * air_density * (W_final**2) * B * chord * C_x_final * dr
        dQ_elem = 0.5 * air_density * (W_final**2) * B * chord * C_y_final * r * dr
        
        total_thrust_fan += dT_elem
        total_torque += dQ_elem

    # --- 4. 総合性能の計算 ---
    
    # パワー P = Torque * omega
    power_watts = total_torque * omega
    
    # ダクトを含めた総推力
    # T_total = T_fan + T_duct
    # OptDuct理論: T_total = T_fan / F_lip
    
    if prop.duct_length > 0.0 and F_lip < 1.0:
        total_thrust_combined = total_thrust_fan / F_lip
        thrust_duct = total_thrust_combined - total_thrust_fan
    else:
        total_thrust_combined = total_thrust_fan
        thrust_duct = 0.0

    # 効率 (Figure of Merit for Hover)
    # FM = (T^1.5 / sqrt(2 * rho * A)) / P
    area_disk = np.pi * (R**2)
    if power_watts > 0:
        fom = (total_thrust_combined**1.5 / np.sqrt(2 * air_density * area_disk)) / power_watts
    else:
        fom = 0.0

    return total_thrust_combined, total_thrust_fan, thrust_duct, total_torque, power_watts, fom

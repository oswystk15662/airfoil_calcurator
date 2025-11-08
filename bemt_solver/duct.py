# bemt_solver/duct.py
import numpy as np
from .geometry import Propeller

def calculate_wake_contraction(prop: Propeller):
    """
    OptDuct論文(図5.1-4)に基づき、ダクト形状から後流の収縮係数(k^2)を推定する。
    [修正] リップ半径(R_lip)の影響を考慮する。
    
    Args:
        prop (Propeller): プロペラ形状オブジェクト

    Returns:
        k_squared (float): 収縮係数 (S_fan / S_wake)
    """
    
    if prop.duct_length <= 0.0:
        # ダクトなし (プロペラ単体) -> OptDuct論文の理論値 k^2 = 2.0
        return 2.0

    # D/L (直径/長さ) 比を計算
    d_over_l = prop.diameter / prop.duct_length
    
    # --- OptDuct論文 (JAXA-RM-21-006) 図5.1-4 のデータをデジタル化 ---
    
    # X軸: Diameter of Duct / Length of Duct (d/L)
    dl_ratio_data = np.array([0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0])
    
    # Y軸 (リップなし: Without Lip) [cite: 599]
    wake_diam_ratio_no_lip = np.array([0.71, 0.76, 0.84, 0.90, 0.93, 0.945, 0.955, 0.97, 0.975]) 
    
    # Y軸 (リップあり: With Lip, R/D=0.031) 
    wake_diam_ratio_with_lip = np.array([0.72, 0.83, 0.89, 0.92, 0.94, 0.955, 0.965, 0.975, 0.98]) 
    
    # -----------------------------------------------------------------
    
    # リップなし(0)とリップあり(1)の補間関数を作成
    interp_no_lip = np.interp(d_over_l, dl_ratio_data, wake_diam_ratio_no_lip, right=1.0)
    interp_with_lip = np.interp(d_over_l, dl_ratio_data, wake_diam_ratio_with_lip, right=1.0)
    
    # 現在のリップ半径比 (R_lip / D) を計算
    current_lip_ratio = prop.duct_lip_radius / prop.diameter
    
    # 論文の基準値 (R/D = 0.031) に対して、現在のリップがどれくらいの効果を持つか (0.0～1.0)
    # 0.031 を超えるリップは、0.031 と同じ効果とみなす (クリップ)
    lip_effect_factor = np.clip(current_lip_ratio / 0.031, 0.0, 1.0)
    
    # 「リップなし」と「リップあり」の間を線形補間
    wake_diam_ratio = (1.0 - lip_effect_factor) * interp_no_lip + lip_effect_factor * interp_with_lip
    
    # k = D_duct / D_wake
    k = 1.0 / wake_diam_ratio
    
    # k^2 = S_duct / S_wake
    k_squared = k**2
    
    # k^2は最大でも2.0 (ダクトなし)、最小で1.0 (無限長ダクト)
    return np.clip(k_squared, 1.0, 2.0)

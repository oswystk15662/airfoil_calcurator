# bemt_solver/duct.py
import numpy as np
from .geometry import Propeller

def calculate_wake_contraction(prop: Propeller):
    """
    OptDuct論文(図5.1-4)に基づき、ダクト形状から後流の収縮係数(k^2)を推定する。
    
    Args:
        prop (Propeller): プロペラ形状オブジェクト

    Returns:
        k_squared (float): 収縮係数 (S_fan / S_wake)
    """
    
    if prop.duct_length <= 0.0:
        # ダクトなし (プロペラ単体) -> OptDuct論文の理論値 k^2 = 2.0 [cite: 558]
        return 2.0

    # D/L (直径/長さ) 比を計算
    d_over_l = prop.diameter / prop.duct_length
    
    # OptDuct論文 (JAXA-RM-21-006) の図5.1-4 "With Lip" のカーブをデジタル化
    # X軸: Diameter of Duct / Length of Duct (d/L)
    # Y軸: Diameter of Trailing Vortex Ring / Diameter of Duct (D_wake / D_duct)
    
    # 論文のグラフ  から読み取ったデータポイント
    # (x = d/L, y = D_wake/D_duct)
    dl_ratio_data = np.array([0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0])
    wake_diam_ratio_data = np.array([0.71, 0.83, 0.89, 0.92, 0.94, 0.955, 0.965, 0.975, 0.98]) 
    
    # プロペラ単体 (d/L=0) の理論値 (D_wake/D_duct = 1/sqrt(2) approx 0.707) [cite: 558]
    # OptDuctのグラフもこれに漸近している [cite: 494]
    
    # D/L比が1.0を超える場合は、収縮はほぼないと仮定 (比率=1.0)
    dl_ratio_data_full = np.append(dl_ratio_data, [d_over_l, 100.0])
    wake_diam_ratio_data_full = np.append(wake_diam_ratio_data, [np.interp(d_over_l, dl_ratio_data, wake_diam_ratio_data), 1.0])
    
    # D/L比から後流の直径比 (D_wake / D_duct) を線形補間で求める
    wake_diam_ratio = np.interp(d_over_l, dl_ratio_data_full, wake_diam_ratio_data_full)
    
    # k = D_duct / D_wake
    k = 1.0 / wake_diam_ratio
    
    # k^2 = S_duct / S_wake
    k_squared = k**2
    
    # k^2は最大でも2.0 (ダクトなし)、最小で1.0 (無限長ダクト) [cite: 558]
    return np.clip(k_squared, 1.0, 2.0)
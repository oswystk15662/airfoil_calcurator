# bemt_solver/losses.py
import numpy as np

def prandtl_loss_factor(r: float, hub_radius: float, tip_radius: float,
                        num_blades: int, phi_rad: float) -> float:
    """
    プラントルの翼端および翼根の損失係数 (F) を計算する。
    
    Args:
        r (float): 現在の要素の半径 (m)
        hub_radius (float): ハブ半径 (m)
        tip_radius (float): チップ半径 (m)
        num_blades (int): ブレード枚数
        phi_rad (float): 流入角 (ラジアン)

    Returns:
        float: 損失係数 F (0 < F <= 1.0)
    """
    if phi_rad < 1e-6:
        # ゼロ除算を避ける (流入角が0の場合、推力も0になるためF=1でも問題ない)
        return 1.0

    # --- 翼端損失 ---
    f_tip = (num_blades / 2.0) * (tip_radius - r) / (r * np.sin(phi_rad))
    # f_tip が大きすぎると exp(-f_tip) が 0 になり、arccos(0) = pi/2 となる
    # 浮動小数点誤差を避けるため、f_tipが極端に大きい場合はF_tip=1とする
    if f_tip > 50.0:
        F_tip = 1.0
    else:
        F_tip = (2.0 / np.pi) * np.arccos(np.exp(-f_tip))

    # --- 翼根損失 ---
    f_hub = (num_blades / 2.0) * (r - hub_radius) / (hub_radius * np.sin(phi_rad))
    if f_hub > 50.0:
        F_hub = 1.0
    else:
        F_hub = (2.0 / np.pi) * np.arccos(np.exp(-f_hub))

    # 総合損失係数
    F = F_tip * F_hub
    
    # 計算結果がnanまたは0になるのを防ぐ (最小値を設定)
    if np.isnan(F) or F < 1e-4:
        return 1e-4
        
    return F
# bemt_solver/losses.py
import numpy as np

def prandtl_tip_loss(B, r, R, phi):
    """
    プラントルの翼端損失係数 (Prandtl Tip Loss Factor)
    
    Args:
        B (int): ブレード枚数
        r (float): 現在の半径位置
        R (float): 翼端半径 (Tip Radius)
        phi (float): 流入角 (Inflow Angle) [rad]
        
    Returns:
        F_tip (float): 翼端損失係数 (0.0 < F <= 1.0)
    """
    # ゼロ除算回避 (sin(phi)が0に近い場合)
    sin_phi = np.sin(phi)
    if abs(sin_phi) < 1e-6:
        return 1.0
        
    # 翼端での距離 (R-r) が0になると f -> 0 となり指数関数が発散するので微小値を足すかクリップ
    # 通常は (R-r) をそのまま計算して問題ないが、r=Rのときは注意
    
    exponent = -(B / 2.0) * ((R - r) / (r * sin_phi))
    
    # exp(exponent) は exponentが負の大きな値になると0に近づく
    # F = (2/pi) * arccos(exp(exponent))
    
    # 数値安定性のため、exponentの範囲を制限しても良いが、通常はこのままでOK
    # ただし、arccosの引数が1を超えないようにクリップが必要
    
    arg = np.exp(exponent)
    arg = np.clip(arg, 0.0, 1.0)
    
    F_tip = (2.0 / np.pi) * np.arccos(arg)
    
    # Fは0以下にはならないが、念のためクリップ
    return np.clip(F_tip, 1e-6, 1.0)

def prandtl_hub_loss(B, r, R_hub, phi):
    """
    プラントルのハブ損失係数 (Prandtl Hub Loss Factor)
    
    Args:
        B (int): ブレード枚数
        r (float): 現在の半径位置
        R_hub (float): ハブ半径
        phi (float): 流入角 [rad]
        
    Returns:
        F_hub (float): ハブ損失係数
    """
    sin_phi = np.sin(phi)
    if abs(sin_phi) < 1e-6:
        return 1.0
        
    exponent = -(B / 2.0) * ((r - R_hub) / (R_hub * sin_phi))
    
    arg = np.exp(exponent)
    arg = np.clip(arg, 0.0, 1.0)
    
    F_hub = (2.0 / np.pi) * np.arccos(arg)
    
    return np.clip(F_hub, 1e-6, 1.0)

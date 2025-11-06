# airfoil_database.py
import numpy as np
from scipy.interpolate import interp1d

"""
これはXFOIL/XFLR5で事前計算した翼型データベースの「モック（模擬）」です。
実際には、ここで複数のレイノルズ数に対応したCSVやJSONを読み込み、
scipy.interpolate.interpn などでRe数に対しても補間を行います。

ここでは簡略化のため、Re数の影響は無視し、
2種類の翼型（厚い/薄い）の性能差のみを定義します。
"""

# --- 翼型1: 根本用 (厚い・高揚力型) ---
# 迎角(deg)
AOA_THICK = np.array([-5.0,  0.0,   5.0,  10.0,  15.0,  20.0])
# 揚力係数
CL_THICK  = np.array([-0.2,  0.4,   0.9,   1.3,   1.1,   0.9])
# 抗力係数 (低Reなので高め)
CD_THICK  = np.array([0.05,  0.03,  0.04,  0.08,  0.15,  0.25])

# --- 翼型2: 翼端用 (薄い・高速型) ---
# 迎角(deg)
AOA_THIN  = np.array([-5.0,  0.0,   5.0,  10.0,  15.0,  20.0])
# 揚力係数
CL_THIN   = np.array([-0.1,  0.2,   0.7,   1.1,   1.0,   0.8])
# 抗力係数 (薄いので抗力が低い)
CD_THIN   = np.array([0.04,  0.02,  0.03,  0.07,  0.14,  0.24])

# --- 補間オブジェクトの作成 ---
# fill_value="extrapolate" は、指定範囲外の迎角が来ても外挿して値を返す
cl_interp_thick = interp1d(AOA_THICK, CL_THICK, kind='linear', fill_value="extrapolate")
cd_interp_thick = interp1d(AOA_THICK, CD_THICK, kind='linear', fill_value="extrapolate")

cl_interp_thin = interp1d(AOA_THIN, CL_THIN, kind='linear', fill_value="extrapolate")
cd_interp_thin = interp1d(AOA_THIN, CD_THIN, kind='linear', fill_value="extrapolate")

# --- データベース辞書 ---
AIRFOIL_DB = {
    "LOW_RE_THICK": (cl_interp_thick, cd_interp_thick),
    "LOW_RE_THIN": (cl_interp_thin, cd_interp_thin)
}

# --- 外部から呼び出す関数 ---

def get_airfoil_performance(airfoil_name: str, reynolds: float, aoa_deg: float):
    """
    データベースから翼型性能を取得する。
    (このモックでは Reynolds は無視する)
    """
    if airfoil_name not in AIRFOIL_DB:
        raise ValueError(f"翼型 {airfoil_name} はデータベースに存在しません。")

    cl_func, cd_func = AIRFOIL_DB[airfoil_name]
    
    cl = float(cl_func(aoa_deg))
    cd = float(cd_func(aoa_deg))
    
    # 簡易モデル（失速）
    # 迎角が大きすぎたら性能を悪化させる
    if aoa_deg > 18.0 or aoa_deg < -8.0:
        cl *= 0.5 # 揚力低下
        cd *= 3.0 # 抗力増大
    
    return cl, cd, 0.0 # (cm は 0.0 を返す)
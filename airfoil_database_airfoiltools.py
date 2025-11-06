# airfoil_database_airfoiltools.py
import numpy as np
import pandas as pd
from scipy.interpolate import RegularGridInterpolator
import io

print("--- [DEBUG] Loading Airfoil Database (AirfoilTools/XFLR5 Edition) ---")

# --- データベースのシミュレーション ---
# 本来、このデータは外部のCSVファイルから読み込みます。
# (例: pd.read_csv("airfoil_data/s1223/re_10000.csv"))
#
# ここでは、CSVファイルの中身を文字列としてハードコードし、
# データベースが構築されるプロセスをシミュレートします。

# --- S1223 (低Re, 高揚力型) ---
# Re=10k, Re=50k の2つのデータポイントを模倣
S1223_RE10K_CSV = """
AoA,CL,CD,CM
-5.0,-0.20,0.08,0.0
0.0,0.50,0.05,0.0
5.0,1.10,0.06,0.0
10.0,1.50,0.10,0.0
15.0,1.30,0.20,0.0
"""
S1223_RE50K_CSV = """
AoA,CL,CD,CM
-5.0,-0.15,0.06,0.0
0.0,0.60,0.04,0.0
5.0,1.25,0.05,0.0
10.0,1.70,0.08,0.0
15.0,1.50,0.18,0.0
"""

# --- E61 (低Re, 標準型) ---
E61_RE10K_CSV = """
AoA,CL,CD,CM
-5.0,-0.10,0.06,0.0
0.0,0.30,0.04,0.0
5.0,0.80,0.05,0.0
10.0,1.10,0.10,0.0
15.0,0.90,0.22,0.0
"""
E61_RE50K_CSV = """
AoA,CL,CD,CM
-5.0,-0.05,0.04,0.0
0.0,0.40,0.03,0.0
5.0,0.90,0.04,0.0
10.0,1.25,0.09,0.0
15.0,1.10,0.20,0.0
"""

# データベースに登録する翼型と、そのCSVデータ
AIRFOIL_FILES = {
    "S1223": {
        10000: S1223_RE10K_CSV,
        50000: S1223_RE50K_CSV
    },
    "E61": {
        10000: E61_RE10K_CSV,
        50000: E61_RE50K_CSV
    }
}

# 補間オブジェクトをグローバルに保存する辞書
# _airfoil_interpolators["S1223"] = (cl_interpolator, cd_interpolator)
_airfoil_interpolators = {}


def _load_airfoil_data():
    """
    起動時に一度だけ実行され、CSVデータを読み込み、
    2D補間オブジェクト (Re, AoA) -> (CL, CD) を作成する。
    """
    for airfoil_name, files_dict in AIRFOIL_FILES.items():
        
        reynolds_numbers = sorted(files_dict.keys())
        
        # 基準となるAoA（すべてのReで共通である必要がある）
        # 最初のファイル (re=10k) のAoAを基準とする
        df_base = pd.read_csv(io.StringIO(files_dict[reynolds_numbers[0]]))
        aoas = df_base['AoA'].values
        
        num_aoas = len(aoas)
        num_res = len(reynolds_numbers)
        
        # データを格納する2Dグリッド (形状: [AoAの数, Reの数])
        cl_grid = np.zeros((num_aoas, num_res))
        cd_grid = np.zeros((num_aoas, num_res))
        
        for i_re, re in enumerate(reynolds_numbers):
            csv_data = files_dict[re]
            df = pd.read_csv(io.StringIO(csv_data))
            
            # (簡単化のため、AoAの数が一致している前提)
            cl_grid[:, i_re] = df['CL'].values
            cd_grid[:, i_re] = df['CD'].values
            
        # 2D補間オブジェクトを作成
        # (aoas, reynolds_numbers) の2つの軸で補間する
        # fill_value=None, bounds_error=False は、補間範囲外の値を
        # 最も近いデータで代用（外挿）することを意味する
        cl_interpolator = RegularGridInterpolator(
            (aoas, reynolds_numbers), cl_grid, 
            bounds_error=False, fill_value=None
        )
        cd_interpolator = RegularGridInterpolator(
            (aoas, reynolds_numbers), cd_grid, 
            bounds_error=False, fill_value=None
        )
        
        # 辞書に保存
        _airfoil_interpolators[airfoil_name] = (cl_interpolator, cd_interpolator)
        print(f"  [DB] Loaded '{airfoil_name}' (Re: {reynolds_numbers})")

# --- メイン関数 (BEMTソルバーから呼び出される) ---
def get_airfoil_performance(airfoil_name: str, reynolds: float, aoa_deg: float):
    """
    データベースから翼型性能を2D補間して取得する。
    
    Args:
        airfoil_name (str): 翼型名 (e.g., "S1223")
        reynolds (float): レイノルズ数
        aoa_deg (float): 迎角 (度)
        
    Returns:
        (cl, cd, cm)
    """
    if airfoil_name not in _airfoil_interpolators:
        # データベースにない翼型が呼ばれた
        print(f"Warning: Airfoil '{airfoil_name}' not found in DB. Using fallback.")
        cl, cd = (0.1, 0.2) # 悪い性能を返す
    
    else:
        cl_func, cd_func = _airfoil_interpolators[airfoil_name]
        
        # 補間点 (AoA, Re)
        point = np.array([aoa_deg, reynolds])
        
        # 補間を実行
        cl = float(cl_func(point))
        cd = float(cd_func(point))
        
        # 補間結果が物理的におかしい場合（XFOIL失敗時の対策）
        if np.isnan(cl) or np.isnan(cd) or cd <= 0:
            cl, cd = (0.1, 0.2) # 悪い性能を返す

    return cl, cd, 0.0 # (cm は 0.0 を返す)

# --- 起動時にデータベースを初期化 ---
_load_airfoil_data()
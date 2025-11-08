# airfoil_database_airfoiltools.py
import numpy as np
import pandas as pd
from scipy.interpolate import RegularGridInterpolator
import os
import glob # ファイル検索用

print("--- [DEBUG] Loading Airfoil Database (Robust Interpolation V2) ---")

# 読み込む翼型データが保存されている場所
CSV_DIR = "airfoil_data/csv_polars"

# 補間オブジェクトをグローバルに保存する辞書
# _airfoil_interpolators["S1223"] = (cl_interpolator, cd_interpolator)
_airfoil_interpolators = {}

# --- [修正点 1] ---
# 私たちがデータベースに欲しい「標準の」迎角グリッドを定義する
# (generate_database.py で指定した範囲と一致させる)
STANDARD_AOAS = np.arange(-5.0, 15.0 + 0.5, 0.5) # -5.0, -4.5, ..., 14.5, 15.0 (計41点)

# -------------------


def _load_airfoil_data():
    """
    起動時に一度だけ実行され、CSV_DIR にあるCSVデータをすべて読み込み、
    2D補間オブジェクト (Re, AoA) -> (CL, CD) を作成する。
    不完全なCSVファイル（XFOILが途中で失敗したもの）も処理可能。
    """
    
    csv_files = glob.glob(os.path.join(CSV_DIR, "*.csv"))
    
    if not csv_files:
        print(f"Warning: No CSV files found in {CSV_DIR}. Database is empty.")
        print("Please run 'generate_database.py' first.")
        return

    airfoils_data = {} 
    
    for f_path in csv_files:
        filename = os.path.basename(f_path)
        try:
            parts = filename.replace(".csv", "").split("_re_")
            airfoil_name = parts[0].lower() # ◀ 小文字に統一
            re = int(parts[1])
            
            if airfoil_name not in airfoils_data:
                airfoils_data[airfoil_name] = {}
                
            # --- [修正点 2] ---
            # CSVを読み込むが、中身が空でないかチェック
            df = pd.read_csv(f_path)
            if df.empty or len(df) < 2:
                # print(f"Info: Skipping empty/invalid file: {filename}")
                continue # データが少なすぎるファイルは無視
            # --- [修正点 2ここまで] ---

            airfoils_data[airfoil_name][re] = df
            
        except Exception as e:
            print(f"Warning: Could not parse filename {filename}: {e}")
            continue
            
    # --- [修正点 3] 補間オブジェクトの作成ロジックを変更 ---
    for airfoil_name, re_data_dict in airfoils_data.items():
        
        reynolds_numbers = sorted(re_data_dict.keys())
        if len(reynolds_numbers) < 2:
            print(f"Warning: '{airfoil_name}' needs at least 2 valid Re data files for interpolation. Skipping.")
            continue
            
        num_aoas = len(STANDARD_AOAS)
        num_res = len(reynolds_numbers)
        
        cl_grid = np.zeros((num_aoas, num_res))
        cd_grid = np.zeros((num_aoas, num_res))
        
        # データを2Dグリッドに配置
        for i_re, re in enumerate(reynolds_numbers):
            df = re_data_dict[re]
            
            # XFOILのCSVからAoAとCL/CDを読み込む
            csv_aoas = df['AoA'].values
            csv_cls = df['CL'].values
            csv_cds = df['CD'].values
            
            # 1D補間を実行し、不完全なデータを標準グリッド(STANDARD_AOAS)にマッピング
            # np.interp はソート済みの入力を期待するため、念のためソートする
            sort_indices = np.argsort(csv_aoas)
            csv_aoas_sorted = csv_aoas[sort_indices]
            csv_cls_sorted = csv_cls[sort_indices]
            csv_cds_sorted = csv_cds[sort_indices]
            
            # 標準グリッドに補間
            # left/rightは、標準AoAがCSVの範囲外だった場合に使う値
            cl_interpolated = np.interp(STANDARD_AOAS, csv_aoas_sorted, csv_cls_sorted, left=np.nan, right=np.nan)
            cd_interpolated = np.interp(STANDARD_AOAS, csv_aoas_sorted, csv_cds_sorted, left=np.nan, right=np.nan)

            cl_grid[:, i_re] = cl_interpolated
            cd_grid[:, i_re] = cd_interpolated

        # 2D補間オブジェクトを作成
        cl_interpolator = RegularGridInterpolator(
            (STANDARD_AOAS, reynolds_numbers), cl_grid, 
            bounds_error=False, fill_value=None # fill_value=None は外挿を意味する
        )
        cd_interpolator = RegularGridInterpolator(
            (STANDARD_AOAS, reynolds_numbers), cd_grid, 
            bounds_error=False, fill_value=None
        )
        
        _airfoil_interpolators[airfoil_name] = (cl_interpolator, cd_interpolator)
        print(f"  [DB] Loaded '{airfoil_name}' (Re: {reynolds_numbers})")
    # --- [修正点 3 ここまで] ---


# --- メイン関数 (BEMTソルバーから呼び出される) ---
def get_airfoil_performance(airfoil_name: str, reynolds: float, aoa_deg: float):
    """
    データベースから翼型性能を2D補間して取得する。
    """
    
    airfoil_name_lower = airfoil_name.lower()
    
    if airfoil_name_lower not in _airfoil_interpolators:
        # print(f"Warning: Airfoil '{airfoil_name}' not found. Available: {list(_airfoil_interpolators.keys())}")
        cl, cd = (0.1, 0.2) # 悪い性能を返す
    
    else:
        cl_func, cd_func = _airfoil_interpolators[airfoil_name_lower]
        
        point = np.array([aoa_deg, reynolds])
        
        cl = float(cl_func(point))
        cd = float(cd_func(point))
        
        # 補間結果が NaN (データ欠損) の場合
        if np.isnan(cl) or np.isnan(cd) or cd <= 0:
            # print(f"Warning: Interpolation failed for {airfoil_name_lower} @ Re={reynolds}, AoA={aoa_deg}. Using fallback.")
            cl, cd = (0.1, 0.2) # 悪い性能を返す

    return cl, cd, 0.0 # (cm は 0.0 を返す)

# 🔽 [新規追加] Optunaから利用可能な翼型リストを取得する関数 🔽
def get_available_airfoils():
    """
    データベースのロードに成功した翼型のリストを返す。
    """
    if not _airfoil_interpolators:
        # このスクリプトがインポートされた時点で _load_airfoil_data() が
        # 実行されているはずだが、念のため呼び出す
        _load_airfoil_data()
        
    return list(_airfoil_interpolators.keys())
# 🔼 [新規追加] 🔼

# --- 起動時にデータベースを初期化 ---
_load_airfoil_data()
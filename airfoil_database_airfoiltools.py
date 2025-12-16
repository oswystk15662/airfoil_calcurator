# airfoil_database_airfoiltools.py
import numpy as np
import pandas as pd
from scipy.interpolate import RegularGridInterpolator
import os
import glob 

print("--- [DEBUG] Loading Airfoil Database (Robust Interpolation V2 + Thickness) ---")

# --- 設定 ---
CSV_DIR = "airfoil_data/csv_polars"

# --- 翼型の最大厚み率 (t/c) データベース ---
# (Airfoil Toolsなどを参照して設定)
AIRFOIL_THICKNESS_DB = {
    "ah-7-47-6": 0.06,  "ah79100a": 0.10, "aqilla": 0.088,
    "clearky": 0.117, "clarky": 0.117,
    "dae11": 0.09, "dae21": 0.09, "dae31": 0.09, "dae51": 0.09,
    "e193": 0.102, "e395": 0.12, "e423": 0.125, "e61": 0.056,
    "fx60-100": 0.10, "fx63-137": 0.137,
    "geminism": 0.09, "gm15": 0.138,
    "goe430": 0.088, "goe501": 0.129, "goe795": 0.13,
    "mh32": 0.09, "mh114": 0.13,
    "naca4412": 0.12, "naca6409": 0.09,
    "s1210": 0.12, "s1223": 0.121, "s4083": 0.12, "s8035": 0.14,
    "sd7032-099-88": 0.10, "sd7037-092-88": 0.09
}

# 補間器をキャッシュする辞書
_airfoil_interpolators = {}
_loaded = False

# XFOIL計算に使用したAoAリスト (CSVの構造チェック用)
STANDARD_AOAS = np.arange(-5.0, 15.5, 0.5)

def _load_airfoil_data():
    global _loaded
    if _loaded:
        return

    csv_files = glob.glob(os.path.join(CSV_DIR, "*.csv"))
    if not csv_files:
        print(f"Warning: No CSV files found in {CSV_DIR}. Please run generate_database.py.")
        return

    for f_path in csv_files:
        filename = os.path.basename(f_path)
        try:
            # ファイル名から翼型名とRe数を抽出 (例: s1223_re_10000.csv)
            parts = filename.replace(".csv", "").split("_re_")
            if len(parts) != 2:
                continue
                
            airfoil_name = parts[0]
            re_val = float(parts[1])
            
            # CSV読み込み
            df = pd.read_csv(f_path)
            
            # 必須カラムチェック
            if 'AoA' not in df.columns:
                continue
            if df.empty or len(df) < 2:
                continue

            # 辞書に格納 (まだ補間器は作らない。データを集めるだけ)
            if airfoil_name not in _airfoil_interpolators:
                _airfoil_interpolators[airfoil_name] = {"re_data": []}
            
            # データを辞書に追加
            # { 're': 10000, 'df': DataFrame }
            _airfoil_interpolators[airfoil_name]["re_data"].append({
                're': re_val,
                'df': df
            })

        except Exception as e:
            print(f"Error loading {filename}: {e}")
            continue

    # データを集め終わったら、翼型ごとに2D補間器を作成
    loaded_count = 0
    
    for name, data in _airfoil_interpolators.items():
        re_list_data = data["re_data"]
        # Re数でソート
        re_list_data.sort(key=lambda x: x['re'])
        
        # 2Dデータ配列の準備 (Re方向, AoA方向)
        # RegularGridInterpolatorを使うには、格子点が揃っている必要がある。
        # しかしXFOILの結果は収束失敗などでAoAが欠けている場合がある。
        # そこで、共通のAoA軸 (STANDARD_AOAS) に各Reのデータを1D補間(resample)して揃える。
        
        valid_re_values = []
        cl_grid = []
        cd_grid = []
        
        for item in re_list_data:
            df = item['df']
            re_val = item['re']
            
            # データが少なすぎるReはスキップ
            if len(df) < 5: continue
            
            # 重複AoAの削除
            df = df.drop_duplicates(subset=['AoA'])
            
            # 1D補間関数の作成 (このReにおける CL(alpha), CD(alpha))
            # bounds_error=False, fill_value=None (外挿はしない、端点を使う)
            cl_interp_1d = np.interp(STANDARD_AOAS, df['AoA'], df['CL'], left=np.nan, right=np.nan)
            cd_interp_1d = np.interp(STANDARD_AOAS, df['AoA'], df['CD'], left=np.nan, right=np.nan)
            
            # NaNが含まれる場合はそのReデータを採用しない（グリッドが作れないため）
            # あるいは、もっと狭いAoA範囲に制限する手もあるが、今回はスキップ
            if np.isnan(cl_interp_1d).any():
                # print(f"  [Info] Skipping Re={re_val} for {name} due to incomplete AoA coverage.")
                continue
                
            valid_re_values.append(re_val)
            cl_grid.append(cl_interp_1d)
            cd_grid.append(cd_interp_1d)
            
        if len(valid_re_values) < 2:
            # print(f"  [Warning] Not enough valid Re data for {name}. Skipping.")
            del _airfoil_interpolators[name] # 登録解除
            continue

        # 2Dグリッド作成
        cl_grid = np.array(cl_grid) # shape: (num_re, num_aoa)
        cd_grid = np.array(cd_grid)
        
        # 2D補間器の作成
        # 軸: (Re, AoA)
        try:
            interp_cl = RegularGridInterpolator((valid_re_values, STANDARD_AOAS), cl_grid, bounds_error=False, fill_value=None)
            interp_cd = RegularGridInterpolator((valid_re_values, STANDARD_AOAS), cd_grid, bounds_error=False, fill_value=None)
            
            # 辞書を更新 (補間器に置き換え)
            _airfoil_interpolators[name] = (interp_cl, interp_cd)
            # print(f"  [DB] Loaded '{name}' (Re: {valid_re_values})")
            loaded_count += 1
            
        except Exception as e:
            print(f"Error creating interpolator for {name}: {e}")
            del _airfoil_interpolators[name]

    print(f"--- Database Loaded: {loaded_count} airfoils ready. ---")
    _loaded = True

def get_available_airfoils():
    """ 利用可能な翼型名のリストを返す """
    _load_airfoil_data()
    return list(_airfoil_interpolators.keys())

def get_airfoil_properties(airfoil_name: str, reynolds: float, aoa_deg: float):
    """
    指定された翼型、Re数、迎角における空力係数 (CL, CD) と、
    翼型の厚み率 (t/c) を返す。
    
    Args:
        airfoil_name (str): 翼型名 (例: 's1223')
        reynolds (float): レイノルズ数
        aoa_deg (float): 迎角 (度)

    Returns:
        cl (float): 揚力係数
        cd (float): 抗力係数
        t_c (float): 最大厚み率 (Thickness-to-Chord Ratio)
    """
    _load_airfoil_data()
    
    airfoil_name_lower = airfoil_name.lower()
    
    # 1. 厚み率 (t/c) の取得
    # データベースになければデフォルト値 10% (0.10) を返す
    t_c_ratio = AIRFOIL_THICKNESS_DB.get(airfoil_name_lower, 0.10)

    # 2. 空力係数 (CL, CD) の取得
    if airfoil_name_lower not in _airfoil_interpolators:
        # データがない場合はペナルティ値を返す (最適化で選ばれないようにする)
        # print(f"Warning: Airfoil '{airfoil_name}' not in database.")
        return 0.001, 0.5, t_c_ratio # 低揚力、高抗力
    
    cl_func, cd_func = _airfoil_interpolators[airfoil_name_lower]
    
    # 補間実行
    # 外挿(bounds_error=False)の設定により、範囲外は端点の値が返る
    point = np.array([reynolds, aoa_deg])
    
    try:
        cl = float(cl_func(point))
        cd = float(cd_func(point))
    except ValueError:
        return 0.001, 0.5, t_c_ratio

    # 異常値チェック (念のため)
    if np.isnan(cl) or np.isnan(cd):
        return 0.001, 0.5, t_c_ratio
    
    # CDが負になることはあり得ないので修正
    if cd <= 0.0001: cd = 0.0001

    return cl, cd, t_c_ratio
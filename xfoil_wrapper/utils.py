# xfoil_wrapper/utils.py
import os
import re
import pandas as pd # ◀ pandasをインポート

AIRFOIL_DIR = "./xfoil_wrapper/airfoils/" # (これはもう使わないかもしれない)

# ... (既存の find_airfoil_file, generate_xfoil_input_single_aoa, parse_xfoil_output_single は残してOK) ...


# ----------------------------------------------------
# 🔽 [新規追加] XFOILのポーラー出力ファイルをパースする関数 🔽
# ----------------------------------------------------
def parse_xfoil_polar_file(filepath: str):
    """
    XFOILが PACC で保存した .pol ファイル (実体はテキスト) を読み取り、
    クリーンな CSV ファイルとして上書き保存する。
    """
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        # XFOILの出力はヘッダーが12行ある
        header_lines = 12
        data_lines = lines[header_lines:]
        
        # スペース区切りのデータを読み込む
        data = [line.strip().split() for line in data_lines]
        
        # pandas DataFrameに変換
        df = pd.DataFrame(data, columns=['AoA', 'CL', 'CD', 'CDp', 'CM', 'Top_Xtr', 'Bot_Xtr'])
        
        # 必要な列だけ（AoA, CL, CD）を抽出し、数値型に変換
        df_clean = df[['AoA', 'CL', 'CD']].astype(float)
        
        # 元のファイルにクリーンなCSVとして上書き保存
        df_clean.to_csv(filepath, index=False)
        
    except Exception as e:
        print(f"Error parsing XFOIL output file {filepath}: {e}")
        # パースに失敗したら空のファイルを作成
        pd.DataFrame(columns=['AoA', 'CL', 'CD']).to_csv(filepath, index=False)

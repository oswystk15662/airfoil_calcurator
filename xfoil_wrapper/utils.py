# xfoil_wrapper/utils.py
import os
import pandas as pd

def parse_xfoil_polar_file(filepath: str):
    """
    XFOILが PACC で保存した .pol ファイルを読み取り、
    クリーンな CSV ファイル (AoA, CL, CD) として上書き保存する。
    ヘッダー行数に依存せず、数値データのみを抽出するロバストな実装。
    """
    try:
        # ファイルが存在しない場合は空のCSVを作成して終了
        if not os.path.exists(filepath):
            _create_empty_csv(filepath)
            return

        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        valid_data = []
        
        for line in lines:
            parts = line.strip().split()
            
            # データ行は通常、空白区切りで数値が並んでいる
            # 少なくとも3列 (alpha, CL, CD) 必要
            if len(parts) < 3:
                continue
                
            try:
                # 最初の3つが数値に変換できるか試す
                # (ヘッダー行や区切り線はここで ValueError になりスキップされる)
                aoa = float(parts[0])
                cl = float(parts[1])
                cd = float(parts[2])
                
                valid_data.append([aoa, cl, cd])
            except ValueError:
                continue

        # データが見つからなかった場合でも、ヘッダー付きの空ファイルにする
        if not valid_data:
            _create_empty_csv(filepath)
            return

        # DataFrame作成
        df = pd.DataFrame(valid_data, columns=['AoA', 'CL', 'CD'])
        
        # 重複データの削除 (念のため)
        df = df.drop_duplicates(subset=['AoA'])
        
        # CSVとして保存
        df.to_csv(filepath, index=False)
        
    except Exception as e:
        print(f"Error parsing XFOIL output file {filepath}: {e}")
        # エラー時も安全のため空のCSVを作成
        _create_empty_csv(filepath)

def _create_empty_csv(filepath):
    """ヘッダーのみの空CSVを作成するヘルパー関数"""
    try:
        with open(filepath, 'w') as f:
            f.write("AoA,CL,CD\n")
    except:
        pass

# (以下の関数は使用していませんが、互換性のために残してもOKです)
def find_airfoil_file(airfoil_name: str) -> str | None:
    return None # ダミー実装
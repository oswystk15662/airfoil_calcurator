# xfoil_wrapper/core.py
import subprocess
import os
import sys
from . import utils

XFOIL_EXEC_PATH = "./xfoil.exe" 

# Windows用の起動設定 (ちらつき防止)
startupinfo = None
if sys.platform == "win32":
    startupinfo = subprocess.STARTUPINFO()
    startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startupinfo.wShowWindow = subprocess.SW_HIDE

# ... (既存の get_airfoil_performance 関数はそのまま残す) ...

# ----------------------------------------------------
# 🔽 [新規追加] バッチ解析用の関数 🔽
# ----------------------------------------------------

def generate_polar_data(airfoil_name: str, 
                        dat_file_path: str, 
                        reynolds: float, 
                        output_csv_path: str,
                        aoa_start: float = -5.0,
                        aoa_end: float = 15.0,
                        aoa_step: float = 0.5):
    """
    XFOILをバッチモードで実行し、指定したRe数のポーラーカーブをCSVファイルに保存する。
    低Reでの収束性を高めるため、パネル数を増やし粘性計算を初期化する。
    """
    
    # XFOILはパスにスペースがあると失敗することがあるため、
    # 'PACC'コマンドで保存するファイル名を指定する
    
    # XFOILに渡すコマンド文字列を生成
    # 1. 翼型をロード
    # 2. パネル数を増やす (GDES -> PANE -> 250)
    # 3. OPER (操作モード) へ
    # 4. VISC (粘性) モードにし、Re数を指定
    # 5. PACC (ポーラー蓄積) を開始し、保存ファイル名を設定
    # 6. ITER (反復回数) を設定 (例: 100回)
    # 7. ASeq (迎角シーケンス) を実行
    # 8. PACC を終了
    # 9. QUIT
    
    commands = f"""
    LOAD {dat_file_path}
    {airfoil_name}
    GDES
    PANE
    250
    
    OPER
    VISC {reynolds}
    ITER 100
    PACC
    {output_csv_path}
    
    ASEQ {aoa_start} {aoa_end} {aoa_step}
    
    PACC
    
    QUIT
    """
    
    command_input = "\n".join([line.strip() for line in commands.splitlines()])

    try:
        process = subprocess.run(
            [XFOIL_EXEC_PATH],
            input=command_input,
            capture_output=True,
            text=True,
            timeout=60, # 1回のバッチ処理に最大60秒
            encoding='utf-8',
            startupinfo=startupinfo
        )
        
        if process.returncode != 0:
            print(f"  [XFOIL Error] {airfoil_name} @ Re {reynolds:.0f} failed.")
            print(f"  STDOUT: {process.stdout[-500:]}") # エラー出力
            print(f"  STDERR: {process.stderr}")
            return False
        
        # XFOILは 'output.csv' という名前で保存するが、
        # 中身は整形されていないため、パースする必要がある
        utils.parse_xfoil_polar_file(output_csv_path)
        
        return True

    except Exception as e:
        print(f"Error running XFOIL batch: {e}")
        return False

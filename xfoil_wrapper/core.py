# xfoil_wrapper/core.py
import subprocess
import os # ◀ os がインポートされていることを確認
import sys
from . import utils

XFOIL_EXEC_PATH = "./xfoil.exe" 

# Windows用の起動設定 (ちらつき防止)
startupinfo = None
if sys.platform == "win32":
    startupinfo = subprocess.STARTUPINFO()
    startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startupinfo.wShowWindow = subprocess.SW_HIDE

# ... (get_airfoil_performance 関数はおそらく不要ですが、あっても問題ありません) ...

# ----------------------------------------------------
# 🔽 [差し替え] この関数を丸ごと置き換えてください 🔽
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
    (既存ファイルの上書き問題を修正済み)
    """
    
    # --- [修正点 1] XFOIL実行前に、既存の出力ファイルを削除する ---
    # これにより、XFOILが "Set current parameters to old save file values ? y/n>" 
    # という対話的な質問を表示するのを防ぎます。
    if os.path.exists(output_csv_path):
        try:
            os.remove(output_csv_path)
        except OSError as e:
            print(f"Warning: Could not remove old file {output_csv_path}. {e}")
    # --- [修正点 1 ここまで] ---
    
    # XFOILに渡すコマンド文字列を生成
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
        
        # --- [修正点 2] 成功した場合 (returncode 0) のみパースする ---
        if process.returncode == 0 and os.path.exists(output_csv_path):
            # XFOILは .pol ファイルを生成する
            # (utils.py の関数でこれをクリーンなCSVに変換)
            utils.parse_xfoil_polar_file(output_csv_path)
            return True
        else:
            # XFOILが失敗した場合
            print(f"  [XFOIL Error] {airfoil_name} @ Re {reynolds:.0f} failed.")
            if process.stderr:
                print(f"  STDERR: {process.stderr}")
            else:
                 print(f"  STDOUT (last 500 chars): {process.stdout[-500:]}")
            return False
        # --- [修正点 2 ここまで] ---

    except Exception as e:
        print(f"Error running XFOIL batch: {e}")
        return False

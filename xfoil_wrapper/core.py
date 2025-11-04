# xfoil_wrapper/core.py
import subprocess
import os, sys
from . import utils  # 同一モジュール内のutilsをインポート

# XFOIL実行ファイルのパス（環境に合わせて設定）
XFOIL_EXEC_PATH = "./xfoil.exe" 

# --- 🔽 [修正点 3] Windows用のフラグを追加 🔽 ---
# Windows specific: Hide the console window
CREATE_NO_WINDOW_FLAG = 0
if sys.platform == "win32":
    CREATE_NO_WINDOW_FLAG = 0x08000000
# --- 🔼 [修正点 3] 🔼 ---

# --- 🔽 [修正点 1] Windows用の起動設定を追加 🔽 ---
startupinfo = None
if sys.platform == "win32":
    startupinfo = subprocess.STARTUPINFO()
    startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startupinfo.wShowWindow = subprocess.SW_HIDE # これがウィンドウを非表示にする
# --- 🔼 [修正点 1] 🔼 ---

def get_airfoil_performance(airfoil_name: str, reynolds: float, aoa: float):
    """
    指定した翼型、レイノルズ数、迎角（単一）に対する性能（CL, CD）を取得する。
    BEMT計算の反復ループ内で使うことを想定。

    Returns:
        (cl, cd, cm) or (None, None, None) if calculation fails
    """
    
    # 1. 翼型座標ファイルのパスを取得
    airfoil_file_path = utils.find_airfoil_file(airfoil_name)
    if not airfoil_file_path:
        print(f"Error: Airfoil file for {airfoil_name} not found.")
        return None, None, None

    # 2. XFOILに渡すバッチコマンド文字列を生成
    #    (例: "LOAD {filepath}\nOPER\nVISC {reynolds}\nASEQ {aoa} {aoa} 1\n..." )
    xfoil_commands = utils.generate_xfoil_input_single_aoa(
        airfoil_file_path, reynolds, aoa
    )

    # # --- 🔽 [デバッグ] ここから追加 🔽 ---
    # print("--- [Debug] XFOIL Input Commands ---")
    # print(repr(xfoil_commands)) # コマンド文字列（改行含む）を正確に表示
    # print("------------------------------------")
    # # --- 🔼 [デバッグ] ここまで追加 🔼 ---

    # 3. subprocessでXFOILを実行
    try:
        process = subprocess.run(
            [XFOIL_EXEC_PATH],
            input=xfoil_commands,
            capture_output=True,
            text=True,
            timeout=10, # 計算が終わらない場合に備えてタイムアウト
            encoding='utf-8',
            # creationflags=CREATE_NO_WINDOW_FLAG,  # ◀ [修正点 4] この行を追加
            startupinfo=startupinfo  # ◀ [修正点 2] この行を追加
        )
        
        # 4. XFOILの標準出力をパースして結果を取得
        # # --- 🔽 [デバッグ] ここから変更 🔽 ---
        # print("--- [Debug] XFOIL STDOUT ---")
        # print(process.stdout)
        # print("------------------------------")
        # print("--- [Debug] XFOIL STDERR ---")
        # print(process.stderr) # 標準エラー出力を表示
        # print("------------------------------")

        cl, cd, cm = utils.parse_xfoil_output_single(process.stdout)
        
        # --- 🔽 [修正点 5] デバッグprintを削除 🔽 ---
        # ログが [Debug] Parsing failed. (CL is None) で
        # 埋まってしまうため、このデバッグは削除します。
        # if cl is None:
        #     print("[Debug] Parsing failed. (CL is None)")
        # --- 🔼 [修正点 5] 🔼 ---
        
        return cl, cd, cm
        # --- 🔼 [デバッグ] ここまで変更 🔼 ---

    except Exception as e:
        print(f"Error running XFOIL: {e}")
        return None, None, None

def get_polar(airfoil_name: str, reynolds: float, aoa_start: float, aoa_end: float, aoa_step: float):
    """
    指定した迎角範囲のポーラーカーブ（CL, CDのリスト）を取得する。
    （こちらは解析用。BEMT計算では上記 single_aoa の方が使いやすい）
    """
    # 処理は get_airfoil_performance と同様だが、
    # utils.generate_xfoil_input_polar(...) を呼び出す
    # utils.parse_xfoil_output_polar(...) で結果をパースする
    pass
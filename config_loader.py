import yaml
import os
import sys

# デフォルトの設定ファイルパス
DEFAULT_CONFIG_PATH = "config.yaml"

def load_config(config_path=None):
    """
    指定されたパスからYAML設定ファイルを読み込む。
    パスが指定されない場合はデフォルトの 'config.yaml' を探す。
    """
    if config_path is None:
        config_path = DEFAULT_CONFIG_PATH

    if not os.path.exists(config_path):
        print(f"Error: Configuration file '{config_path}' not found.")
        print("Please create a 'config.yaml' file in the project root.")
        sys.exit(1)

    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        return config
    except Exception as e:
        print(f"Error reading YAML file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    # テスト用
    conf = load_config()
    print("Config loaded successfully.")
    print(f"Project: {conf.get('project', {}).get('name')}")
# bemt_solver/geometry.py
import numpy as np

class Propeller:
    """
    プロペラの幾何学的形状を定義するクラス。
    (ダクト形状情報も含む)
    """
    def __init__(self,
                 hub_radius: float,
                 tip_radius: float,
                 num_blades: int,
                 r_coords: np.ndarray,
                 pitch_coords_deg: np.ndarray,
                 chord_coords: np.ndarray,
                 airfoil_name: str,
                 duct_length: float = 0.0,   # ◀ 追加
                 duct_lip_radius: float = 0.0 # ◀ 追加
                 ):
        """
        Args:
            hub_radius (float): ハブ半径 (m)
            tip_radius (float): チップ半径 (m)
            num_blades (int): ブレード枚数
            r_coords (np.ndarray): 形状定義点 (半径位置) の配列 (m)
            pitch_coords_deg (np.ndarray): r_coordsに対応するピッチ角の配列 (度)
            chord_coords (np.ndarray): r_coordsに対応するコード長の配列 (m)
            airfoil_name (str): 使用する翼型名 (xfoil_wrapperが認識する名前)
            duct_length (float): ダクト長さ (m)  [ステップ3で追加]
            duct_lip_radius (float): ダクトのリップ半径 (m) [ステップ3で追加]
        """
        self.hub_radius = hub_radius
        self.tip_radius = tip_radius
        self.num_blades = num_blades
        self.airfoil_name = airfoil_name
        
        # --- 🔽 [追加] 🔽 ---
        self.diameter = tip_radius * 2.0
        self.duct_length = duct_length
        self.duct_lip_radius = duct_lip_radius
        # --- 🔼 [追加] 🔼 ---

        # 補間用にデータを保持
        self._r_coords = r_coords
        self._pitch_coords_deg = pitch_coords_deg
        self._chord_coords = chord_coords

    def get_pitch_deg(self, r: float) -> float:
        """指定した半径 r でのピッチ角 (度) を補間して取得"""
        return float(np.interp(r, self._r_coords, self._pitch_coords_deg))

    def get_chord(self, r: float) -> float:
        """指定した半径 r でのコード長 (m) を補間して取得"""
        return float(np.interp(r, self._r_coords, self._chord_coords))
"""
Pinocchio-basierter Collision Handler für Kinova Gen3.

Führt Selbstkollisionsprüfungen und Umgebungskollisionsprüfungen
(z. B. Tisch-Schutz bei Z = 0.05m und Raum-Hindernisse) im Worldspace durch.
"""
import time
import numpy as np

try:
    import pinocchio as pin
    _PINOCCHIO_AVAILABLE = True
except ImportError:
    _PINOCCHIO_AVAILABLE = False


class PinocchioCollisionHandler:
    """
    Collision Handler mit Pinocchio Geometry Models.
    Prüft Selbstkollisionen der Armsegmente, Tisch-, Wand- und Objektkollisionen.
    """

    def __init__(
        self,
        model=None,
        geom_model=None,
        table_height_m: float = 0.05,
        wall_back_x_m: float = -0.25,
        wall_left_y_m: float = -0.85,
        wall_right_y_m: float = 0.85
    ):
        self.model = model
        self.geom_model = geom_model
        self.geom_data = None
        self.data = None

        # =====================================================================
        # TODO: [LABOR-UMGEBUNG] EINSTELLUNGEN FÜR DECKENMONTAGE & KOLLISIONEN
        #
        # KOORDINATENSYSTEM DER ROBOTERBASIS (base_link / SOCKEL):
        #   -X = HINTEN  (Seite MIT Powerknopf, LED-Ring und LAN-Kabel)
        #   +X = VORNE   (Gegenüberliegende glatte Seite OHNE Knöpfe)
        #   +Y = LINKS   (Wenn du auf den Powerknopf blickst: deine rechte Hand = Links vom Arm)
        #   -Y = RECHTS  (Wenn du auf den Powerknopf blickst: deine linke Hand = Rechts vom Arm)
        #   +Z = DECKE   (nach oben zur Deckenhalterung)
        #   -Z = BODEN   (nach unten zum Tisch/Boden)
        # =====================================================================
        self.is_ceiling_mounted = True       # True = Arm hängt an der Decke nach unten!

        # TODO: [LABOR] Gemessenen Abstand zur Decke & zum Tisch/Boden eintragen
        self.ceiling_z_max = 0.05            # Decke über der Basis (Z > 0.05m ist Deckenstruktur)
        self.floor_table_z_min = -1.30       # Boden/Tisch unter dem Arm (z. B. -1.30m unter der Decke)

        # TODO: [LABOR] Wandabstände für alle 4 Richtungen eintragen:
        self.wall_back_x  = wall_back_x_m    # Wand HINTEN (-X, hinter Powerknopf/Kabel) (z. B. -0.25m)
        self.wall_front_x = 0.85             # Wand VORNE  (+X, vor glatter Seite) (z. B. +0.85m)
        self.wall_left_y  = wall_left_y_m    # Wand LINKS  (+Y) (z. B. +0.30m = 30cm links vom Sockel)
        self.wall_right_y = wall_right_y_m   # Wand RECHTS (-Y) (z. B. -0.30m = 30cm rechts vom Sockel)

        # =====================================================================
        # TODO: [LABOR] HIER FESTE LABOR-GEGENSTÄNDE (MONITORE, STATIVE) EINTRAGEN
        # Format: {"name": "Monitor", "x": 0.40, "y": 0.30, "z": -0.60, "radius": 0.20}
        # =====================================================================
        self.static_obstacles: list[dict] = [
            # {"name": "Tisch_Labor", "x": 0.30, "y": 0.0, "z": -1.10, "radius": 0.40},
            # {"name": "Monitor", "x": 0.40, "y": 0.50, "z": -0.50, "radius": 0.20},
        ]

        # Dynamische Hindernisse aus dem Kamera-Feed (Weg 2)
        self.dynamic_obstacles: dict[str, dict] = {}

        self.enabled = _PINOCCHIO_AVAILABLE and (model is not None)
        if self.enabled:
            self._init_collision_models()

    def update_dynamic_obstacle(self, obstacle_id: str, x: float, y: float, z: float, radius: float = 0.20):
        """
        Aktualisiert die 3D-Position eines dynamisch erkannten Hindernisses/AprilTags (Weg 2).
        """
        self.dynamic_obstacles[obstacle_id] = {
            "name": obstacle_id,
            "x": x,
            "y": y,
            "z": z,
            "radius": radius,
            "stamp": time.time()
        }

    def add_obstacle(self, name: str, x: float, y: float, z: float, radius: float = 0.15):
        """Fügt ein neues festes Hindernis im Worldspace hinzu."""
        self.static_obstacles.append({
            "name": name, "x": x, "y": y, "z": z, "radius": radius
        })

    def set_models(self, model, geom_model=None):
        """Setzt oder aktualisiert die Pinocchio Kinematik- und Geometriemodelle."""
        self.model = model
        self.geom_model = geom_model
        self.enabled = _PINOCCHIO_AVAILABLE and (model is not None)
        if self.enabled:
            self._init_collision_models()

    def _init_collision_models(self):
        """Initialisiert Data-Strukturen und Kollisionspaare."""
        if self.model:
            self.data = self.model.createData()
            if self.geom_model:
                self.geom_data = self.geom_model.createData()
                try:
                    self.geom_model.addAllCollisionPairs()
                except Exception:
                    pass

    def is_configuration_valid(self, q: list[float]) -> tuple[bool, str]:
        """
        Prüft eine Gelenkkonfiguration q auf:
          1. Decken- & Boden-Kollision (Deckenmontage)
          2. Wand-Kollisionsgrenzen (Rückwand & Seitenwände)
          3. Statische & Dynamische Objekte/Hindernisse (Weg 1 & Weg 2)
          4. Pinocchio Selbstkollisionen aller Armsegmente

        Returns: (is_valid: bool, reason: str)
        """
        if not self.enabled or self.model is None:
            return True, "Pinocchio Collision Handler nicht aktiv (Modell nicht geladen)"

        q_full = pin.neutral(self.model)
        q_arr = np.array(q, dtype=np.float64)
        q_full[:min(len(q_arr), self.model.nq)] = q_arr[:min(len(q_arr), self.model.nq)]

        # 1. Vorwärtskinematik & Frame-Placements aktualisieren
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)

        # Liste aller aktiven Hindernisse zusammenstellen (statisch + dynamisch < 3.0s alt)
        now = time.time()
        active_obstacles = list(self.static_obstacles)
        for obs_id, obs in list(self.dynamic_obstacles.items()):
            if now - obs.get("stamp", 0) <= 3.0:
                active_obstacles.append(obs)

        # 2. Tisch-, Decken-, Wand- & Objekt-Kollisionsprüfung im Worldspace
        for frame_id in range(self.model.nframes):
            pos = self.data.oMf[frame_id].translation
            frame_name = self.model.frames[frame_id].name

            # =================================================================
            # A) DECKEN- & BODEN-SCHUTZ (DECKENMONTAGE)
            # =================================================================
            if self.is_ceiling_mounted:
                if pos[2] > self.ceiling_z_max:
                    return False, f'Decken-Kollision droht: Frame "{frame_name}" bei Z={pos[2]:.3f}m > Max Z={self.ceiling_z_max:.2f}m (Decke)'
                if pos[2] < self.floor_table_z_min:
                    return False, f'Boden/Tisch-Kollision droht: Frame "{frame_name}" bei Z={pos[2]:.3f}m < Min Z={self.floor_table_z_min:.2f}m (Boden)'
            else:
                if pos[2] < self.table_height:
                    return False, f'Tisch-Kollision droht: Frame "{frame_name}" bei Z={pos[2]:.3f}m < Min Z={self.table_height:.2f}m'

            # =================================================================
            # B) WAND-SCHUTZ (ALLE 4 RICHTUNGEN)
            # =================================================================
            if pos[0] < self.wall_back_x:
                return False, f'Rückwand-Kollision droht: Frame "{frame_name}" bei X={pos[0]:.3f}m < Min X={self.wall_back_x:.2f}m'
            if pos[0] > self.wall_front_x:
                return False, f'Vorderwand-Kollision droht: Frame "{frame_name}" bei X={pos[0]:.3f}m > Max X={self.wall_front_x:.2f}m'
            if pos[1] < self.wall_left_y:
                return False, f'Seitenwand-Kollision (links) droht: Frame "{frame_name}" bei Y={pos[1]:.3f}m < Min Y={self.wall_left_y:.2f}m'
            if pos[1] > self.wall_right_y:
                return False, f'Seitenwand-Kollision (rechts) droht: Frame "{frame_name}" bei Y={pos[1]:.3f}m > Max Y={self.wall_right_y:.2f}m'

            # =================================================================
            # C) STATISCHE & DYNAMISCHE OBJEKTE (WEG 1 & WEG 2)
            # =================================================================
            for obs in active_obstacles:
                dx = pos[0] - obs["x"]
                dy = pos[1] - obs["y"]
                dz = pos[2] - obs["z"]
                dist = np.sqrt(dx*dx + dy*dy + dz*dz)
                if dist < obs["radius"]:
                    return False, f'Hindernis-Kollision mit "{obs["name"]}" droht! Frame "{frame_name}" ist nur {dist:.2f}m entfernt (Min Radius: {obs["radius"]:.2f}m)'

        # 3. Pinocchio Selbstkollisionsprüfung (falls Geometriemodell geladen)
        if self.geom_model and self.geom_data:
            try:
                pin.computeCollisions(self.model, self.data, self.geom_model, self.geom_data, q_full, True)
                for k in range(len(self.geom_model.collisionPairs)):
                    pair = self.geom_model.collisionPairs[k]
                    res = self.geom_data.collisionResults[k]
                    if res.isCollision():
                        obj1 = self.geom_model.geometryObjects[pair.first].name
                        obj2 = self.geom_model.geometryObjects[pair.second].name
                        return False, f'Selbstkollision erkannt zwischen "{obj1}" und "{obj2}"'
            except Exception as e:
                pass  # Fallback bei reinem URDF-Kinematikmodell

        return True, "OK (Kollisionsfrei)"

"""
Pinocchio-basierter Inverse Kinematics (IK) Solver für Kinova Gen3 (6-DoF).

Unterstützt das Laden der URDF direkt aus dem ROS2 Topic /robot_description
oder aus einer URDF-Datei / XML-String.
"""

import math
import numpy as np

try:
    import pinocchio as pin
    _PINOCCHIO_AVAILABLE = True
except ImportError:
    _PINOCCHIO_AVAILABLE = False


from .collision_handler import PinocchioCollisionHandler


class KinovaIKSolver:
    """
    Inverse Kinematics Solver mit Pinocchio (Levenberg-Marquardt / CLIK Algorithmus)
    und integriertem PinocchioCollisionHandler.
    """

    def __init__(self, urdf_xml_or_path: str = None, end_effector_frame: str = "end_effector_link"):
        self.is_available = _PINOCCHIO_AVAILABLE
        self.model = None
        self.data = None
        self.ee_frame_id = None
        self.collision_handler = PinocchioCollisionHandler()

        if not _PINOCCHIO_AVAILABLE:
            print("[KinovaIKSolver] WARNUNG: Pinocchio-Bibliothek nicht installiert (pip install pinocchio).")
            return

        if urdf_xml_or_path:
            self.load_urdf(urdf_xml_or_path, end_effector_frame)

    def load_urdf(self, urdf_xml_or_path: str, end_effector_frame: str = "end_effector_link") -> bool:
        """Lädt das Robotermodell aus einer URDF-Datei oder einem XML-String."""
        if not _PINOCCHIO_AVAILABLE:
            return False

        try:
            if urdf_xml_or_path.strip().startswith("<"):
                # XML String
                self.model = pin.buildModelFromXML(urdf_xml_or_path)
            else:
                # Dateipfad
                self.model = pin.buildModelFromUrdf(urdf_xml_or_path)

            self.data = self.model.createData()

            # Collision Handler mit aktuellem Modell initialisieren
            self.collision_handler.set_models(self.model)

            # End-Effektor Frame suchen oder Fallback auf bekannte Frames
            target_frame_names = [end_effector_frame, "end_effector_link", "tool_frame", "dummy_tcp", "gripper_base_link"]
            found_frame = None
            for fname in target_frame_names:
                if fname and self.model.existFrame(fname):
                    self.ee_frame_id = self.model.getFrameId(fname)
                    found_frame = fname
                    break

            if found_frame is None:
                self.ee_frame_id = self.model.nframes - 1
                found_frame = self.model.frames[self.ee_frame_id].name

            print(
                f"[KinovaIKSolver] URDF geladen. Gelenke (nq={self.model.nq}), "
                f"End-Effektor Frame ID: {self.ee_frame_id} ('{found_frame}')"
            )

            return True
        except Exception as e:
            print(f"[KinovaIKSolver] Fehler beim Laden der URDF: {e}")
            return False

    def get_forward_kinematics(self, q: list[float]) -> tuple[np.ndarray, np.ndarray] | None:
        """
        Berechnet die Vorwärtskinematik (Position und Rotationsmatrix des End-Effektors)
        für gegebene Gelenkwinkel q.
        """
        if not self.is_available or self.model is None:
            return None

        try:
            q_full = pin.neutral(self.model)
            q_arr = np.array(q, dtype=np.float64)
            q_full[:min(len(q_arr), self.model.nq)] = q_arr[:min(len(q_arr), self.model.nq)]

            pin.forwardKinematics(self.model, self.data, q_full)
            pin.updateFramePlacements(self.model, self.data)

            pos = self.data.oMf[self.ee_frame_id].translation.copy()
            rot = self.data.oMf[self.ee_frame_id].rotation.copy()
            return pos, rot
        except Exception as e:
            print(f"[KinovaIKSolver] Fehler bei Vorwärtskinematik: {e}")
            return None

    def solve_position(
        self,
        x: float,
        y: float,
        z: float,
        q_init: list[float] = None,
        max_iter: int = 500,
        eps: float = 1e-3,
        damp: float = 1e-5,
    ) -> list[float] | None:
        """
        Berechnet 6 Gelenkwinkel für eine 3D-Zielposition (x, y, z) in Metern.
        Stellt sicher, dass Punkte außerhalb des erreichbaren Radius auf den maximalen
        Arbeitsradius skaliert werden (inspiriert von ControlModule.TagApproachNode).
        """
        if not self.is_available or self.model is None:
            return None

        # Max. sichere Reichweite (0.85m) - analog zu WorkspaceParameters in ControlModule
        dist = math.sqrt(x*x + y*y + z*z)
        MAX_SAFE_REACH_M = 0.85

        if dist > MAX_SAFE_REACH_M:
            scale = MAX_SAFE_REACH_M / dist
            print(
                f"[KinovaIKSolver] Ziel ({x:+.3f}, {y:+.3f}, {z:+.3f}) m ist {dist:.3f}m entfernt "
                f"→ Skaliere Vektor auf max. Reichweite {MAX_SAFE_REACH_M:.2f}m: "
                f"({x*scale:+.3f}, {y*scale:+.3f}, {z*scale:+.3f}) m"
            )
            x, y, z = x * scale, y * scale, z * scale

        target_pos = np.array([x, y, z])
        return self.solve_se3(target_pos, target_rot=None, q_init=q_init, max_iter=max_iter, eps=eps, damp=damp)

    def solve_se3(
        self,
        target_pos: np.ndarray,
        target_rot: np.ndarray = None,
        q_init: list[float] = None,
        max_iter: int = 500,
        eps: float = 1e-4,
        damp: float = 1e-6,
    ) -> list[float] | None:
        """
        Levenberg-Marquardt Inverse Kinematics für SE3 (Position + Orientierung) mit Multi-Seed Fallback.
        """
        if not self.is_available or self.model is None:
            return None

        # Liste von Startkonfigurationen für Multi-Seed Fallback
        seeds = []
        if q_init is not None:
            q_arr = np.array(q_init, dtype=np.float64)
            q_first = pin.neutral(self.model)
            q_first[:min(len(q_arr), self.model.nq)] = q_arr[:min(len(q_arr), self.model.nq)]
            seeds.append(q_first)

        # Hinzufügen von neutralen und ausgestreckten Fallback-Startwerten
        seeds.append(pin.neutral(self.model))
        
        # Hinzufügen einer leicht gestreckten Startkonfiguration
        q_stretched = pin.neutral(self.model)
        if len(q_stretched) >= 6:
            q_stretched[1] = 0.5   # Gelenk 2 leicht anwinkeln
            q_stretched[2] = 1.0   # Gelenk 3 nach vorne strecken
        seeds.append(q_stretched)

        for seed in seeds:
            q = seed.copy()
            for i in range(max_iter):
                pin.forwardKinematics(self.model, self.data, q)
                pin.updateFramePlacements(self.model, self.data)

                current_pos = self.data.oMf[self.ee_frame_id].translation
                err_pos = target_pos - current_pos

                if target_rot is not None:
                    current_rot = self.data.oMf[self.ee_frame_id].rotation
                    err_rot = pin.log3(current_rot.T @ target_rot)
                    err = np.hstack([err_pos, err_rot])
                else:
                    err = err_pos

                if np.linalg.norm(err) < eps:
                    joint_angles = q[:6].tolist()
                    is_valid, reason = self.collision_handler.is_configuration_valid(joint_angles)
                    if not is_valid:
                        print(f"[KinovaIKSolver] IK-Lösung verworfen wegen Kollision: {reason}")
                        break  # Nächsten Seed versuchen
                    return joint_angles

                # Jacobian berechnen
                J = pin.computeFrameJacobian(
                    self.model, self.data, q, self.ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
                )
                if target_rot is None and J.ndim == 2:
                    J = J[:3, :]  # Nur Positionskomponenten

                # Wichtig: Nur die 6 Arm-Gelenke bewegen (Greifer-Gelenke sperren)
                if J.shape[1] > 6:
                    J[:, 6:] = 0.0

                # Damped Pseudo-Inverse (Levenberg-Marquardt Schritt)
                JJt = J @ J.T + damp * np.eye(J.shape[0])
                dq = J.T @ np.linalg.solve(JJt, err)

                q = pin.integrate(self.model, q, dq)

        print("[KinovaIKSolver] IK Konvergenz nicht innerhalb max_iter erreicht.")
        return None


class IKMovement:
    """
    Mixin-Klasse für KinovaMover.
    Enthält alle High-Level Bewegungsfunktionen, die auf dem Pinocchio IK-Solver basieren.
    """

    def move_to_cartesian_position(
        self,
        x: float,
        y: float,
        z: float,
        duration: int = 20,
        q_init: list[float] | None = None
    ) -> bool:
        """
        Berechnet per Pinocchio Inverse Kinematics (IK) die Gelenkwinkel für
        die 3D-Zielkoordinaten (x, y, z) in Metern im Base-Frame und bewegt den Arm dorthin.
        """
        if not self.ik_solver.is_available or self.ik_solver.model is None:
            self.get_logger().error(
                'IK-Solver nicht bereit! Stelle sicher, dass pinocchio installiert ist '
                'und die URDF auf /robot_description publiziert wurde.'
            )
            return False

        if q_init is None and hasattr(self, '_current_oriented_position'):
            q_init = self._current_oriented_position

        joint_angles = self.ik_solver.solve_position(x, y, z, q_init=q_init)
        if joint_angles is None:
            self.get_logger().error(f'Keine IK-Lösung für Position ({x:.2f}, {y:.2f}, {z:.2f}) m gefunden.')
            return False

        self.get_logger().info(f'IK-Lösung für ({x:.2f}, {y:.2f}, {z:.2f}) m gefunden → fahre Ziel an.')
        self._current_oriented_position = joint_angles
        self.move_arm_to(joint_angles, duration=duration)
        return True

    def move_to_arm_camera_tag_ik(self, tag_id: int = 3, duration: int = 10, offset_z: float = 0.15) -> bool:
        """
        [ARM-KAMERA / WORLDSPACE IK]
        Nutzt TagWorld / WorldSpaceNode, um die normalisierte 3D-Position des AprilTags
        direkt im Base-Frame (base_link) zu beziehen und per Pinocchio IK anzufahren.
        """
        if not self.ik_solver.is_available or self.ik_solver.model is None:
            self.get_logger().error(
                'IK-Solver nicht bereit! Stelle sicher, dass pinocchio installiert ist '
                'und die URDF auf /robot_description publiziert wurde.'
            )
            return False

        # 1. Versuche TagWorld aus WorldSpaceNode zu nutzen
        tag_pos = None
        if hasattr(self, 'tag_world') and self.tag_world and self.tag_world.fresh:
            tag_pos = self.tag_world.position(tag_id)
            if tag_pos is None:
                visible_ids = self.tag_world.tag_ids()
                if len(visible_ids) > 0:
                    tag_id = visible_ids[0]
                    tag_pos = self.tag_world.position(tag_id)

        # 2. Fallback auf _worldspace_tag_poses
        if tag_pos is None:
            world_poses = getattr(self, '_worldspace_tag_poses', {})
            if tag_id in world_poses:
                tag_pos = world_poses[tag_id]
            elif len(world_poses) > 0:
                tag_id = list(world_poses.keys())[0]
                tag_pos = world_poses[tag_id]

        # 3. Falls noch kein Tag im Worldspace registriert, kurz auf Event warten
        if tag_pos is None and hasattr(self, '_tag_event'):
            self.get_logger().info('[IK-Anfahrt] Warte auf WorldSpace AprilTag Position auf /vision/tags...')
            self._tag_event.clear()
            self._tag_event.wait(timeout=3.0)

            world_poses = getattr(self, '_worldspace_tag_poses', {})
            if tag_id in world_poses:
                tag_pos = world_poses[tag_id]
            elif len(world_poses) > 0:
                tag_id = list(world_poses.keys())[0]
                tag_pos = world_poses[tag_id]

        if tag_pos is None:
            self.get_logger().warn(f'[IK-Anfahrt] Kein AprilTag (ID {tag_id}) im Base-Frame auf /vision/tags empfangen!')
            return False

        x_base, y_base, z_base = float(tag_pos[0]), float(tag_pos[1]), float(tag_pos[2])
        # Bei Deckenmontage (+Z nach unten): Ein Offset "über dem Tag" bedeutet näher zur Decke (- offset_z)
        # Ist offset_z positiv übergeben, ziehen wir ihn ab, um sich dem Roboterfuß anzunähern statt tiefer zu steuern.
        z_target = z_base - abs(offset_z) if z_base > 0 and offset_z > 0 else z_base + offset_z

        self.get_logger().info(
            f'[IK-Anfahrt] AprilTag {tag_id} im Base-Frame: x={x_base:+.3f}m, y={y_base:+.3f}m, z={z_base:+.3f}m '
            f'→ Berechne IK (Ziel Z: {z_target:+.3f}m, Dauer: {duration}s)...'
        )

        return self.move_to_cartesian_position(x_base, y_base, z_target, duration=duration)

    # Alias für Abwärtskompatibilität
    move_to_tag_ik = move_to_arm_camera_tag_ik

    def move_to_worldspace_tag_ik(self, tag_id: int = 3, duration: int = 10, offset_z: float = 0.15) -> bool:
        """
        Bewegt den Arm per Pinocchio IK direkt zu einem Tag aus dem WorldSpaceNode (/vision/tags).
        """
        return self.move_to_arm_camera_tag_ik(tag_id=tag_id, duration=duration, offset_z=offset_z)

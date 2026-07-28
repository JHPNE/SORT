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

            # End-Effektor Frame suchen oder Fallback auf den letzten Frame
            if self.model.existFrame(end_effector_frame):
                self.ee_frame_id = self.model.getFrameId(end_effector_frame)
            else:
                self.ee_frame_id = self.model.nframes - 1

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
        max_iter: int = 100,
        eps: float = 1e-4,
        damp: float = 1e-6,
    ) -> list[float] | None:
        """
        Berechnet 6 Gelenkwinkel für eine 3D-Zielposition (x, y, z) in Metern.
        """
        if not self.is_available or self.model is None:
            return None

        target_pos = np.array([x, y, z])
        return self.solve_se3(target_pos, target_rot=None, q_init=q_init, max_iter=max_iter, eps=eps, damp=damp)

    def solve_se3(
        self,
        target_pos: np.ndarray,
        target_rot: np.ndarray = None,
        q_init: list[float] = None,
        max_iter: int = 100,
        eps: float = 1e-4,
        damp: float = 1e-6,
    ) -> list[float] | None:
        """
        Levenberg-Marquardt Inverse Kinematics für SE3 (Position + Orientierung).
        """
        if not self.is_available or self.model is None:
            return None

        q = pin.neutral(self.model)
        if q_init is not None:
            q_arr = np.array(q_init, dtype=np.float64)
            q[:min(len(q_arr), self.model.nq)] = q_arr[:min(len(q_arr), self.model.nq)]

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
                    return None
                return joint_angles

            # Jacobian berechnen
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            if target_rot is None and J.ndim == 2:
                J = J[:3, :]  # Nur Positionskomponenten

            # Damped Pseudo-Inverse (Levenberg-Marquardt Schritt)
            JJt = J @ J.T + damp * np.eye(J.shape[0])
            dq = J.T @ np.linalg.solve(JJt, err)

            q = pin.integrate(self.model, q, dq)

        print("[KinovaIKSolver] IK Konvergenz nicht innerhalb max_iter erreicht.")
        joint_angles = q[:6].tolist()
        is_valid, reason = self.collision_handler.is_configuration_valid(joint_angles)
        if not is_valid:
            print(f"[KinovaIKSolver] IK-Lösung verworfen wegen Kollision: {reason}")
            return None
        return joint_angles


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
        duration: int = 5,
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

    def move_to_arm_camera_tag_ik(self, duration: int = 5, offset_z: float = 0.15) -> bool:
        """
        [ARM-KAMERA / EYE-IN-HAND]
        Wartet auf die 3D-Position eines AprilTags der Arm-Kamera und bewegt den End-Effektor per Pinocchio IK dorthin.

        TODO: [GREIFEN / OFFSET] offset_z anpassen:
          - 0.15m (15 cm): Sicherheitsabstand / Vorpositionierung vor dem Tag
          - 0.03m - 0.05m (3-5 cm): Greifposition (Greiferfinger umschließen das Objekt)

        :param duration: Bewegungsdauer in Sekunden.
        :param offset_z: Sicherheitsabstand vor dem Tag in Metern (Standard: 0.15m = 15 cm).
        """
        if not hasattr(self, '_tag_event'):
            self.get_logger().error('VisualTracker nicht initialisiert!')
            return False

        if not self.ik_solver.is_available or self.ik_solver.model is None:
            self.get_logger().error(
                'IK-Solver nicht bereit! Stelle sicher, dass pinocchio installiert ist '
                'und die URDF auf /robot_description publiziert wurde.'
            )
            return False

        self._tag_event.clear()
        self.get_logger().info('[ArmKamera-IK] Warte auf AprilTag-Position der Arm-Kamera...')
        tag_found = self._tag_event.wait(timeout=5.0)

        if not tag_found or getattr(self, '_last_position', None) is None:
            self.get_logger().warn('[ArmKamera-IK] Kein AprilTag von Arm-Kamera empfangen!')
            return False

        x, y, z = self._last_position
        self.get_logger().info(
            f'[ArmKamera-IK] Tag-Position im ArmKamera-Frame erkannt: x={x:+.3f}m, y={y:+.3f}m, z={z:+.3f}m → Berechne IK...'
        )

        from .positions import HOME_POSITION
        q_current = getattr(self, '_current_oriented_position', HOME_POSITION)
        fk_res = self.ik_solver.get_forward_kinematics(q_current)

        if fk_res is not None:
            p_ee, R_ee = fk_res
            # Abzug des Sicherheitsabstands offset_z in Sichtrichtung (Kamera-Z-Achse)
            target_cam = np.array([x, y, max(0.05, z - offset_z)])
            target_base = p_ee + R_ee @ target_cam
            target_x, target_y, target_z = float(target_base[0]), float(target_base[1]), float(target_base[2])
            self.get_logger().info(
                f'[ArmKamera-IK] Zielposition im Base-Frame: x={target_x:+.3f}m, y={target_y:+.3f}m, z={target_z:+.3f}m'
            )
            return self.move_to_cartesian_position(target_x, target_y, target_z, duration=duration, q_init=q_current)
        else:
            self.get_logger().warn('[ArmKamera-IK] FK konnte nicht berechnet werden. Nutze Direktkoordinaten.')
            return self.move_to_cartesian_position(x, y, z, duration=duration, q_init=q_current)

    # Alias für Abwärtskompatibilität
    move_to_tag_ik = move_to_arm_camera_tag_ik

    def move_to_worldspace_tag_ik(self, tag_id: int = 3, duration: int = 5, offset_z: float = 0.15) -> bool:
        """
        Bewegt den Arm per Pinocchio IK direkt zu einem Tag aus dem WorldSpaceNode (/vision/tags).

        TODO: [GREIFEN / OFFSET] offset_z anpassen:
          - 0.15m (15 cm): Sicherheitsabstand / Vorpositionierung
          - 0.03m - 0.05m (3-5 cm): Greifposition zum Schließen des Greifers

        :param tag_id: ID des AprilTags (z. B. 3).
        :param duration: Anfahr-Dauer in Sekunden.
        :param offset_z: Sicherheitsabstand vor dem Objekt in Metern.
        """
        world_poses = getattr(self, '_worldspace_tag_poses', {})
        if tag_id not in world_poses:
            self.get_logger().error(f'[WorldSpace-IK] Kein Tag mit ID {tag_id} auf /vision/tags bekannt!')
            return False

        x_base, y_base, z_base = world_poses[tag_id]
        self.get_logger().info(
            f'[WorldSpace-IK] AprilTag {tag_id} im WorldSpace erkannt: '
            f'x={x_base:+.3f}m, y={y_base:+.3f}m, z={z_base:+.3f}m → Berechne IK...'
        )
        return self.move_to_cartesian_position(x_base, y_base, z_base + offset_z, duration=duration)




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


class KinovaIKSolver:
    """
    Inverse Kinematics Solver mit Pinocchio (Levenberg-Marquardt / CLIK Algorithmus).
    """

    def __init__(self, urdf_xml_or_path: str = None, end_effector_frame: str = "end_effector_link"):
        self.is_available = _PINOCCHIO_AVAILABLE
        self.model = None
        self.data = None
        self.ee_frame_id = None

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

            # End-Effektor Frame suchen oder Fallback auf den letzten Frame
            if self.model.existFrame(end_effector_frame):
                self.ee_frame_id = self.model.getFrameId(end_effector_frame)
            else:
                self.ee_frame_id = self.model.nframes - 1

            return True
        except Exception as e:
            print(f"[KinovaIKSolver] Fehler beim Laden der URDF: {e}")
            return False

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

        if q_init is None:
            q = pin.neutral(self.model)
        else:
            q = np.array(q_init, dtype=np.float64)

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
                return q.tolist()

            # Jacobian berechnen
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            if target_rot is None:
                J = J[:3, :]  # Nur Positionskomponenten

            # Damped Pseudo-Inverse (Levenberg-Marquardt Schritt)
            JJt = J @ J.T + damp * np.eye(J.shape[0])
            dq = J.T @ np.linalg.solve(JJt, err)

            q = pin.integrate(self.model, q, dq)

        print("[KinovaIKSolver] IK Konvergenz nicht innerhalb max_iter erreicht.")
        return q.tolist()


class IKMovement:
    """
    Mixin-Klasse für KinovaMover.
    Enthält alle High-Level Bewegungsfunktionen, die auf dem Pinocchio IK-Solver basieren.
    """

    def move_to_cartesian_position(self, x: float, y: float, z: float, duration: int = 5) -> bool:
        """
        Berechnet per Pinocchio Inverse Kinematics (IK) die Gelenkwinkel für
        die 3D-Zielkoordinaten (x, y, z) in Metern und bewegt den Arm dorthin.
        """
        if not self.ik_solver.is_available or self.ik_solver.model is None:
            self.get_logger().error(
                'IK-Solver nicht bereit! Stelle sicher, dass pinocchio installiert ist '
                'und die URDF auf /robot_description publiziert wurde.'
            )
            return False

        joint_angles = self.ik_solver.solve_position(x, y, z)
        if joint_angles is None:
            self.get_logger().error(f'Keine IK-Lösung für Position ({x:.2f}, {y:.2f}, {z:.2f}) m gefunden.')
            return False

        self.get_logger().info(f'IK-Lösung für ({x:.2f}, {y:.2f}, {z:.2f}) m gefunden → fahre Ziel an.')
        self.move_arm_to(joint_angles, duration=duration)
        return True


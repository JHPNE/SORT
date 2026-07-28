"""
Gripper Controller & Pick & Place Module für Kinova Gen3.

Steuert den Robotiq 2F-140 / 2F-85 Greifer über den offiziellen ROS 2 Action Server
/robotiq_gripper_controller/gripper_cmd (control_msgs/action/GripperCommand).
"""

import time
import numpy as np

try:
    from control_msgs.action import GripperCommand
    from rclpy.action import ActionClient
    _GRIPPER_ACTION_AVAILABLE = True
except ImportError:
    _GRIPPER_ACTION_AVAILABLE = False


class GripperController:
    """
    Mixin für KinovaMover zur Steuerung des Greifers per Action Client
    (/robotiq_gripper_controller/gripper_cmd).
    """

    def _init_gripper(self):
        """Initialisiert den ROS 2 Action Client für den Greifer."""
        self._gripper_client = None
        if _GRIPPER_ACTION_AVAILABLE:
            try:
                self._gripper_client = ActionClient(
                    self,
                    GripperCommand,
                    '/robotiq_gripper_controller/gripper_cmd'
                )
            except Exception as e:
                self.get_logger().warn(f'[Greifer] Action Client konnte nicht erstellt werden: {e}')

        self._gripper_state = 'open'

    def send_gripper_command(self, position: float, max_effort: float = 100.0):
        """
        Sendet ein GripperCommand Action Goal an /robotiq_gripper_controller/gripper_cmd.
        position = 0.0 -> geöffnet, position = 0.8 -> geschlossen (gemäß Reference Guide).
        """
        if not _GRIPPER_ACTION_AVAILABLE or self._gripper_client is None:
            self.get_logger().info(f'[Greifer Simulation] GripperCommand (position={position:.2f}, max_effort={max_effort:.1f})')
            time.sleep(1.0)
            return

        try:
            if not self._gripper_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(
                    f'[Greifer] Action Server /robotiq_gripper_controller/gripper_cmd nicht bereit -> Simuliere Kommando (position: {position:.2f}).'
                )
                time.sleep(1.0)
                return

            goal_msg = GripperCommand.Goal()
            goal_msg.command.position = float(position)
            goal_msg.command.max_effort = float(max_effort)

            self._gripper_client.send_goal_async(goal_msg)
            time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f'[Greifer Fehler] Action Goal gescheitert: {e}')

    def open_gripper(self):
        """Öffnet die Greiferfinger (position = 0.0 gemäß Reference Guide)."""
        self.get_logger().info('[Greifer] Öffne Greiferfinger (position: 0.0)...')
        self.send_gripper_command(position=0.0, max_effort=100.0)
        self._gripper_state = 'open'

    def close_gripper(self, position: float = 0.15):
        """
        Schließt die Greiferfinger sanft um das Objekt (Standard: 0.15 = leichtes Andeuten für Tests).

        # =====================================================================
        # TODO: [LABOR / GREIFER-EINSTELLUNG 1] GREIFDRUCK & POSITION ANPASSEN
        #
        # WAS ANPASSEN:
        #   Ändere den Standardwert 'position = 0.15' oben im Funktionskopf:
        #     - position = 0.15  -->  NUR SANFTES ANDEUTEN (Aktueller Test-Modus)
        #     - position = 0.45  -->  HALB GESCHLOSSEN (für große Gegenstände/Becher)
        #     - position = 0.80  -->  VOLLSTÄNDIG FEST GEGRIFFEN (100 % gemäß Guide)
        # =====================================================================
        """
        pos_val = float(np.clip(position, 0.0, 0.80))
        self.get_logger().info(f'[Greifer] Sanftes Test-Schließen / Andeuten (position: {pos_val:.2f}, max_effort: 100.0)...')
        self.send_gripper_command(position=pos_val, max_effort=100.0)
        self._gripper_state = 'closed'

    def pick_tag(
        self,
        tag_id: int = 3,
        approach_offset: float = 0.15,
        grasp_offset: float = 0.03,
        grasp_closure: float = 0.15
    ) -> bool:
        """
        Führt eine vollständige GREIF-SEQUENZ (Pick) für ein Objekt mit AprilTag aus:
          1. Greifer öffnen (position: 0.0)
          2. Vorpositionierung vor dem Tag per IK (approach_offset = 15 cm)
          3. Anfahrt in Greifposition (grasp_offset = 3 cm)
          4. Greifer leicht andeuten (grasp_closure = 0.15 = 15%)
          5. Objekt leicht anheben (+15 cm in Z)

        # =====================================================================
        # TODO: [LABOR / GREIFER-EINSTELLUNG 2] AUTOMATISCHE GREIF-WEITE ANPASSEN
        #
        # WAS ANPASSEN:
        #   Ändere 'grasp_closure: float = 0.15' im Funktionskopf oben auf 0.80:
        #     - grasp_closure = 0.15  -->  Aktueller Test-Modus (nur leichtes Andeuten)
        #     - grasp_closure = 0.80  -->  Echtes festes Greifen im Labor
        # =====================================================================
        """
        self.get_logger().info(f'[PICK & PLACE] Starte Greif-Sequenz (Pick) für AprilTag {tag_id} (Test-Andeuten: {grasp_closure:.2f})...')

        # 1. Greifer öffnen
        self.open_gripper()

        # 2. Vorpositionierung per IK (15 cm Abstand vor dem Objekt)
        self.get_logger().info(f'[PICK Step 1/4] Fahre Vorpositionierung ({approach_offset*100:.0f} cm vor Tag {tag_id})...')
        if not self.move_to_worldspace_tag_ik(tag_id=tag_id, duration=4, offset_z=approach_offset):
            self.get_logger().error('[PICK FEHLER] Vorpositionierung konnte nicht angefahren werden (evtl. Kollision).')
            return False

        time.sleep(0.5)

        # 3. Anfahrt in Greifposition (3 cm Abstand – Greifer umschließt das Objekt)
        self.get_logger().info(f'[PICK Step 2/4] Fahre Greifposition ({grasp_offset*100:.0f} cm am Tag {tag_id})...')
        if not self.move_to_worldspace_tag_ik(tag_id=tag_id, duration=3, offset_z=grasp_offset):
            self.get_logger().error('[PICK FEHLER] Greifposition konnte nicht angefahren werden.')
            return False

        time.sleep(0.5)

        # 4. Greifer schließen
        self.get_logger().info(f'[PICK Step 3/4] Schließe Greifer um das Objekt (position: {grasp_closure:.2f})...')
        self.close_gripper(position=grasp_closure)

        # 5. Objekt nach dem Greifen anheben (+15 cm im Raum)
        self.get_logger().info('[PICK Step 4/4] Hebe gegriffenes Objekt an (+15 cm)...')
        if hasattr(self, '_worldspace_tag_poses') and tag_id in self._worldspace_tag_poses:
            x, y, z = self._worldspace_tag_poses[tag_id]
            self.move_to_cartesian_position(x, y, z + approach_offset, duration=3)

        self.get_logger().info(f'[PICK ERFOLGREICH] AprilTag {tag_id} gegriffen & angehoben!')
        return True

    def place_object(self, target_x: float, target_y: float, target_z: float, approach_offset: float = 0.15) -> bool:
        """
        Führt eine vollständige ABSETZ-SEQUENZ (Place) aus:
          1. Positionierung über dem Zielplatz (target_z + approach_offset)
          2. Absenken auf Absetzhöhe (target_z)
          3. Greifer öffnen (position: 0.0)
          4. Rückzug nach oben (+15 cm in Z)
        """
        self.get_logger().info(f'[PICK & PLACE] Starte Absetz-Sequenz (Place) bei ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}) m...')

        # 1. Vorpositionierung über der Abstellfläche
        self.get_logger().info('[PLACE Step 1/4] Positioniere über Abstellfläche (+15 cm)...')
        if not self.move_to_cartesian_position(target_x, target_y, target_z + approach_offset, duration=4):
            self.get_logger().error('[PLACE FEHLER] Anfahrt über Abstellfläche fehlgeschlagen.')
            return False

        time.sleep(0.5)

        # 2. Absenken auf Zielhöhe
        self.get_logger().info('[PLACE Step 2/4] Senke Objekt auf Abstellfläche ab...')
        if not self.move_to_cartesian_position(target_x, target_y, target_z, duration=3):
            self.get_logger().error('[PLACE FEHLER] Absenken auf Abstellfläche fehlgeschlagen.')
            return False

        time.sleep(0.5)

        # 3. Greifer öffnen (Objekt loslassen)
        self.get_logger().info('[PLACE Step 3/4] Öffne Greifer (position: 0.0)...')
        self.open_gripper()

        # 4. Rückzug nach oben
        self.get_logger().info('[PLACE Step 4/4] Ziehe Arm nach oben zurück (+15 cm)...')
        self.move_to_cartesian_position(target_x, target_y, target_z + approach_offset, duration=3)

        self.get_logger().info('[PLACE ERFOLGREICH] Objekt abgelegt!')
        return True

"""
Combined System Launcher for SORT Project (Kinova Gen3 + Vision Pipeline).

Launches:
  1. vision_module tag_detector
  2. vision_module world_space
  3. KinovaController arm_mover

Cleanly handles Ctrl+C (SIGINT / SIGTERM) to shut down all nodes gracefully.
"""
import sys
import os
import time
import signal
import subprocess

processes = []


def cleanup(signum=None, frame=None):
    print("\n[Launcher] Beende alle ROS 2 Nodes sauber...")
    for p in reversed(processes):
        if p.poll() is None:
            try:
                p.send_signal(signal.SIGINT)
            except Exception:
                pass

    time.sleep(1.2)
    for p in reversed(processes):
        if p.poll() is None:
            try:
                p.terminate()
            except Exception:
                pass

    print("[Launcher] Alle Prozesse beendet.")
    sys.exit(0)


signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)


def main():
    print("=========================================================")
    print("  SORT System Launcher (Kinova Gen3 + Vision Pipeline)")
    print("=========================================================")

    # Umgebungspfade für direkte Ausführung vorbereiten
    repo_root = os.path.dirname(os.path.abspath(__file__))
    env = os.environ.copy()
    pythonpaths = [
        repo_root,
        os.path.join(repo_root, "KinovaController"),
        os.path.join(repo_root, "VisionModule"),
        os.path.join(repo_root, "TopicHandler"),
        env.get("PYTHONPATH", "")
    ]
    env["PYTHONPATH"] = os.pathsep.join(filter(None, pythonpaths))

    # Helper zum Starten von Nodes (ros2 run mit Fallback auf python -m)
    def start_node(pkg_name, exec_name, module_path, name):
        print(f"[Launcher] Starte {name}...")
        # Erst versuchen via ros2 run
        try:
            p = subprocess.Popen(["ros2", "run", pkg_name, exec_name], env=env, cwd=repo_root)
            time.sleep(0.5)
            if p.poll() is None:
                return p
        except Exception:
            pass

        # Fallback: Direkt via Python-Modul ausführen
        print(f"[Launcher] Fallback für {name}: Starte direkt über Python-Modul '{module_path}'...")
        return subprocess.Popen([sys.executable, "-m", module_path], env=env, cwd=repo_root)

    # 1. Tag Detector Node
    p1 = start_node("vision_module", "tag_detector", "vision_module.TagDetectorNode", "1/3 TagDetectorNode")
    processes.append(p1)
    time.sleep(1.0)

    # 2. World Space Node
    p2 = start_node("vision_module", "world_space", "vision_module.WorldSpaceNode", "2/3 WorldSpaceNode")
    processes.append(p2)
    time.sleep(1.0)

    # 3. Kinova Controller Mover Node
    p3 = start_node("kinova_controller", "arm_mover", "KinovaController.main", "3/3 KinovaMover")
    processes.append(p3)

    print("=========================================================")
    print("  Alle Nodes gestartet! Bereit für Befehle.")
    print("  Publiziere Befehle auf /arm/gesture im 2. Terminal.")
    print("  Beenden mit Strg+C (schließt automatisch alle Nodes).")
    print("=========================================================")

    try:
        while True:
            for p in processes:
                if p.poll() is not None:
                    print(f"[Launcher WARNUNG] Ein Node hat sich unerwartet beendet (Exit Code: {p.poll()}).")
            time.sleep(1.0)
    except KeyboardInterrupt:
        cleanup()


if __name__ == "__main__":
    main()

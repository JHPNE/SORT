import argparse
import time

import cv2
import numpy as np

from AprilTagDetector import AprilTagDetector
from CameraInstrinsics import K_from_fov, ZERO_D


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--hfov", type=float, default=60.0,
                    help="nominal horizontal FOV in degrees, used to guess K")
    ap.add_argument("--tag-size", type=float, default=0.055,
                    help="black square edge length in metres")
    ap.add_argument("--family", default="tag36h11")
    return ap.parse_args()


def main():
    args = parse_args()

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera {args.device}")

    ok, frame = cap.read()
    if not ok:
        raise SystemExit("camera opened but gave no frames")
    h, w = frame.shape[:2]
    print(f"capturing at {w}x{h}")

    K = K_from_fov(w, h, args.hfov)
    print("estimated camera matrix (replace with calibrate_camera.py output):")
    print(np.array2string(K, precision=2, suppress_small=True))

    detector = AprilTagDetector(
        tag_family=args.family,
        tag_size_m=args.tag_size,
        camera_matrix=K,
        dist_coeffs=ZERO_D,
    )

    estimate_pose = True
    frame_count, t0, fps = 0, time.time(), 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        detections = detector.detect(frame, estimate_pose=estimate_pose)
        frame = detector.draw(frame, detections)

        frame_count += 1
        if frame_count % 15 == 0:
            now = time.time()
            fps = 15.0 / (now - t0)
            t0 = now

        cv2.putText(frame, f"{len(detections)} tags   {fps:4.1f} fps", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        for det in detections:
            if det.has_pose:
                x, y, z = det.position_m
                r, p, yw = det.euler_deg
                print(f"id {det.tag_id:3d}  x{x:+.3f} y{y:+.3f} z{z:+.3f} m  "
                      f"rpy {r:+6.1f} {p:+6.1f} {yw:+6.1f} deg  "
                      f"rms {det.reprojection_error:.2f} px")

        cv2.imshow("apriltag local test", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("p"):
            estimate_pose = not estimate_pose
            print(f"pose estimation: {estimate_pose}")
        if key == ord("s"):
            name = f"capture_{int(time.time())}.png"
            cv2.imwrite(name, frame)
            print(f"wrote {name}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
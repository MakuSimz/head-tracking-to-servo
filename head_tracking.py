import cv2
import mediapipe as mp
import numpy as np
import math
import serial
import time
from collections import deque

# ---------- CONFIG ----------
SERIAL_PORT = "/dev/ttyUSB1"  # e.g. "COM3" on Windows or "/dev/ttyACM0" on Linux/RPi
BAUD_RATE = 115200
SEND_TO_SERIAL = True  # Set False for testing without Arduino connected

# Mediapipe face mesh settings
MAX_NUM_FACES = 1
MIN_DETECTION_CONFIDENCE = 0.4
MIN_TRACKING_CONFIDENCE = 0.5

# Smoothing settings (moving average window)
SMOOTHING_WINDOW = 5

# Servo mapping (calibration)
# Map measured head angles (degrees) to servo angles (0-180)
# These are example ranges and may need calibration per setup.
YAW_MIN_ANGLE = -25  # head turned left limit (deg)
YAW_MAX_ANGLE = 25  # head turned right limit (deg)
SERVO_YAW_MIN = 30
SERVO_YAW_MAX = 150

PITCH_MIN_ANGLE = -25  # head down (negative) vs up (positive)
PITCH_MAX_ANGLE = 25
SERVO_PITCH_MIN = 40
SERVO_PITCH_MAX = 140

# Smoothing containers
yaw_history = deque(maxlen=SMOOTHING_WINDOW)
pitch_history = deque(maxlen=SMOOTHING_WINDOW)

# ---------- SERIAL SETUP ----------
ser = None
if SEND_TO_SERIAL:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # allow Arduino to reset
        print(f"Opened serial {SERIAL_PORT} @ {BAUD_RATE}")
    except Exception as e:
        print("Warning: Could not open serial port:", e)
        ser = None
        SEND_TO_SERIAL = False

# ---------- MEDIAPIPE SETUP ----------
mp_face = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils

face_mesh = mp_face.FaceMesh(
    static_image_mode=False,
    max_num_faces=MAX_NUM_FACES,
    refine_landmarks=False,
    min_detection_confidence=MIN_DETECTION_CONFIDENCE,
    min_tracking_confidence=MIN_TRACKING_CONFIDENCE,
)

# Useful landmark indices (MediaPipe FaceMesh)
# We use the outer eye corners and a set of nose landmarks to robustly estimate nose position.
LEFT_EYE_OUTER = 33
RIGHT_EYE_OUTER = 263
NOSE_TIP = 1
# choose a small set of nose landmarks to average (to be robust)
NOSE_LANDMARKS = [1, 2, 98, 327]


# ---------- HELPER FUNCTIONS ----------
def normalized_to_point(landmark, image_w, image_h):
    return np.array([landmark.x * image_w, landmark.y * image_h, landmark.z * image_w])


def calc_head_angles(landmarks, img_w, img_h):
    """
    Returns approximate yaw and pitch in degrees.
    Uses 3D landmarks provided by Mediapipe (normalized x,y,z).
    yaw: left/right rotation (+ right)
    pitch: up/down rotation (+ up)
    """
    # get eye outer corners and nose (average of several nose points)
    left_eye = normalized_to_point(landmarks[LEFT_EYE_OUTER], img_w, img_h)
    right_eye = normalized_to_point(landmarks[RIGHT_EYE_OUTER], img_w, img_h)
    nose_points = np.array(
        [normalized_to_point(landmarks[i], img_w, img_h) for i in NOSE_LANDMARKS]
    )
    nose = nose_points.mean(axis=0)

    # midpoint of eyes
    mid_eyes = (left_eye + right_eye) / 2.0

    # vector from mid_eyes to nose
    v = nose - mid_eyes
    # v components: x (right positive), y (down positive in image coordinates), z (depth: negative means out of image)
    # Convert image y to conventional up-positive by negating y
    vx = v[0]
    vy = -v[1]
    vz = v[2]

    # Use arctan2 with z (depth) to get angles relative to camera
    # yaw = arctan2(vx, -vz)  (we invert vz because Mediapipe z is negative when toward camera)
    # pitch = arctan2(vy, -vz)
    # Avoid divide-by-zero by adding tiny epsilon
    eps = 1e-6
    yaw_rad = math.atan2(vx, -vz + eps)
    pitch_rad = math.atan2(vy, -vz + eps)

    yaw_deg = math.degrees(yaw_rad)
    pitch_deg = math.degrees(pitch_rad)

    return yaw_deg, pitch_deg, mid_eyes, nose


def clamp(val, a, b):
    return max(a, min(b, val))


def map_angle_to_servo(
    measured_angle, measured_min, measured_max, servo_min, servo_max
):
    # clamp measured to range, then map linearly
    m = clamp(measured_angle, measured_min, measured_max)
    # normalize 0..1
    norm = (m - measured_min) / (measured_max - measured_min)
    servo = servo_min + norm * (servo_max - servo_min)
    return int(clamp(round(servo), 0, 180))


def send_servo_angles(yaw_servo, pitch_servo):
    """
    Sends string like: "Y090P120\n"
    """
    msg = f"Y{yaw_servo:03d}P{pitch_servo:03d}\n"
    if ser:
        try:
            ser.write(msg.encode("utf-8"))
        except Exception as e:
            print("Serial write error:", e)
    print("->", msg.strip())


# ---------- MAIN LOOP ----------
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("No frame from webcam, exiting")
            break

        img_h, img_w = frame.shape[:2]
        # convert color for mediapipe
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_mesh.process(rgb)

        display_text = ""
        if results.multi_face_landmarks:
            # take first face only
            landmarks = results.multi_face_landmarks[0].landmark

            yaw_deg, pitch_deg, mid_eyes, nose = calc_head_angles(
                landmarks, img_w, img_h
            )

            # smoothing
            yaw_history.append(yaw_deg)
            pitch_history.append(pitch_deg)
            yaw_sm = sum(yaw_history) / len(yaw_history)
            pitch_sm = sum(pitch_history) / len(pitch_history)

            # map to servo ranges
            servo_yaw = map_angle_to_servo(
                yaw_sm, YAW_MIN_ANGLE, YAW_MAX_ANGLE, SERVO_YAW_MIN, SERVO_YAW_MAX
            )
            servo_pitch = map_angle_to_servo(
                pitch_sm,
                PITCH_MIN_ANGLE,
                PITCH_MAX_ANGLE,
                SERVO_PITCH_MIN,
                SERVO_PITCH_MAX,
            )

            # send to Arduino
            if SEND_TO_SERIAL:
                # print("sending to Arduino")
                send_servo_angles(servo_yaw, servo_pitch)

            # annotate frame
            display_text = f"yaw: {yaw_sm:.1f}°, pitch: {pitch_sm:.1f}°  -> servo Y{servo_yaw} P{servo_pitch}"
            cv2.circle(frame, (int(nose[0]), int(nose[1])), 4, (0, 255, 0), -1)
            cv2.circle(frame, (int(mid_eyes[0]), int(mid_eyes[1])), 4, (255, 0, 0), -1)
            mp_drawing.draw_landmarks(
                frame,
                results.multi_face_landmarks[0],
                mp_face.FACEMESH_TESSELATION,
                mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1),
                mp_drawing.DrawingSpec(color=(0, 128, 255), thickness=1),
            )

        else:
            display_text = "No face detected"

        # overlay text
        cv2.putText(
            frame, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2
        )
        cv2.imshow("Head to Servo", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC to quit
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()

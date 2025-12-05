# Head Tracking to Servo

Head tracking demo that turns webcam face orientation into two servo angles (yaw and pitch). Uses MediaPipe Face Mesh to estimate head pose, smooths readings, maps them into calibrated servo ranges, and streams commands to an Arduino-style controller over serial.

## GitHub Description

- Webcam-to-servo head tracking using MediaPipe Face Mesh and Arduino-style serial control.
- Smooths yaw/pitch, maps calibrated ranges to 0–180 servo commands, and visualizes landmarks in real time.
- Works with or without hardware (toggle `SEND_TO_SERIAL`), includes simple calibration knobs for different rigs.

## Quick Start

- Install Python 3.9+ and required packages:
  ```bash
  pip install opencv-python mediapipe numpy pyserial
  ```
- Plug the Arduino (or compatible board) and note its port (e.g., `COM3`, `/dev/ttyACM0`, `/dev/ttyUSB1`).
- Open `head_tracking.py` and update the config block at the top (serial port, baud, smoothing window, angle/servo ranges).
- Run:
  ```bash
  python head_tracking.py
  ```
- Press `ESC` to exit.

## Hardware Notes

- Webcam accessible as device 0; change the index in `cv2.VideoCapture(0)` if needed.
- Two servos driven by an Arduino sketch that understands messages like `Y090P120\n` (yaw then pitch, each 3 digits).
- Calibrate the angle ranges (`YAW_*`, `PITCH_*`) and servo limits (`SERVO_*`) for your rig so that end stops are respected.

## Configuration Reference (`head_tracking.py`)

- `SERIAL_PORT`, `BAUD_RATE`, `SEND_TO_SERIAL`: serial connection to the microcontroller; set `SEND_TO_SERIAL=False` to test without hardware.
- `MAX_NUM_FACES`, `MIN_DETECTION_CONFIDENCE`, `MIN_TRACKING_CONFIDENCE`: MediaPipe Face Mesh knobs for detection/tracking stability.
- `SMOOTHING_WINDOW`: moving-average window length for yaw/pitch smoothing.
- `YAW_MIN_ANGLE`/`YAW_MAX_ANGLE`, `PITCH_MIN_ANGLE`/`PITCH_MAX_ANGLE`: expected head angle bounds (deg) used for mapping to servos.
- `SERVO_YAW_MIN`/`SERVO_YAW_MAX`, `SERVO_PITCH_MIN`/`SERVO_PITCH_MAX`: servo command bounds (0–180) corresponding to the measured angle limits.

## Function-by-Function

- `normalized_to_point(landmark, image_w, image_h)`: converts a MediaPipe normalized landmark into pixel-space x, y, z.
- `calc_head_angles(landmarks, img_w, img_h)`: estimates yaw/pitch (deg) from eye and nose landmarks; returns angles plus mid-eye and nose points for drawing.
- `clamp(val, a, b)`: bounds a value between `a` and `b`.
- `map_angle_to_servo(measured_angle, measured_min, measured_max, servo_min, servo_max)`: clamps a measured angle to its bounds and linearly maps it into a servo command (0–180).
- `send_servo_angles(yaw_servo, pitch_servo)`: formats and sends `Y###P###` over serial if enabled; logs the message.
- Main loop: grabs frames from the webcam, runs Face Mesh, smooths yaw/pitch, maps to servo angles, optionally sends to the controller, overlays landmarks/text, and displays the window until `ESC`.

## Running Headless or Without Hardware

- Set `SEND_TO_SERIAL = False` to skip serial writes.
- Use a virtual camera or sample video by replacing `cv2.VideoCapture(0)` with a file path or different index.

## Troubleshooting

- **No serial connection**: double-check `SERIAL_PORT`, permissions (e.g., add user to `dialout` on Linux), and that the board is flashed with the expected protocol.
- **No webcam feed**: adjust the camera index or ensure another app is not using the camera.
- **Jittery motion**: increase `SMOOTHING_WINDOW`, raise detection/tracking confidences, or widen angle bounds.

## Repository Layout (top-level)

- `head_tracking.py`: main head-to-servo pipeline (described above).
- `move_servo.ino`: Arduino sketch that listens for `YdddPddd` lines over serial, constrains to 0–180, and drives yaw/pitch servos; flash this onto the board.

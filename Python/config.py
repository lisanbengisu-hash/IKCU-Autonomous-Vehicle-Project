# ============================================================
# CARLA BAĞLANTI AYARLARI
# ============================================================

HOST = "localhost"
PORT = 2000
TIMEOUT = 60.0


# ============================================================
# HARİTA / A* ROTA AYARLARI
# ============================================================

WAYPOINT_DISTANCE = 2.0
DRAW_TIME = 30.0

GOAL_TOLERANCE_METERS = 2.5


# ============================================================
# HIZ PLANLAMA
# ============================================================

MAX_ROUTE_SPEED_KMH = 14.0
MIN_CURVE_SPEED_KMH = 5.5
GOAL_APPROACH_SPEED_KMH = 4.0

CURVE_LOOKAHEAD_METERS = 16.0
BUS_STOP_SLOWDOWN_DISTANCE = 20.0


# ============================================================
# PURE PURSUIT
# ============================================================

LOOKAHEAD_MIN = 3.0
LOOKAHEAD_MAX = 7.0

WHEEL_BASE = 2.8

MAX_STEER = 0.75
STEERING_SMOOTHING_ALPHA = 0.25


# ============================================================
# GAZ / FREN
# ============================================================

MAX_THROTTLE = 0.65
MAX_BRAKE = 0.80


# ============================================================
# RGB KAMERA
# ============================================================

RGB_CAMERA_WIDTH = 640
RGB_CAMERA_HEIGHT = 360
RGB_CAMERA_FOV = 90

RGB_CAMERA_X = 3.0
RGB_CAMERA_Z = 2.5

RGB_CAMERA_TICK = 0.05
RGB_CAMERA_GAMMA = 2.2


# ============================================================
# YOLO
# ============================================================

YOLO_MODEL_PATH = "yolo11n.pt"

YOLO_CONFIDENCE = 0.18
YOLO_IMAGE_SIZE = 640

# COCO:
# 0 = person
# 1 = bicycle
# 2 = car
# 3 = motorcycle
# 5 = bus
# 7 = truck

YOLO_ALLOWED_CLASSES = [
    0,
    1,
    2,
    3,
    5,
    7,
]

DETECTION_REQUIRED_HITS = 2
DETECTION_HOLD_FRAMES = 8


# ============================================================
# ARAÇ TAKİBİ
# ============================================================

FOLLOW_FREE_DISTANCE = 26.0
FOLLOW_TARGET_DISTANCE = 18.0
FOLLOW_SLOW_DISTANCE = 13.0
FOLLOW_STOP_DISTANCE = 8.0


# ============================================================
# DURAK
# ============================================================

BUS_STOP_TOLERANCE_METERS = 3.0
BUS_STOP_WAIT_SECONDS = 5.0
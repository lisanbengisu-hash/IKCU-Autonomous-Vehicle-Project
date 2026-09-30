import math
import time
import random

import carla

from ui import choose_start_and_goal
from bus_stops import BUS_STOPS
from route_planner import RoutePlanner
from special_routes import plan_special_route
from vehicle_controller import VehicleController
from vision_system import VisionSystem
from route_bypass import (
    apply_collision_bypass,
    draw_collision_bypass_debug,
)

from spawn_manager import (
    spawn_ego_near_stop,
    destroy_vehicle,
)



# ============================================================
# CARLA AYARLARI
# ============================================================

CARLA_HOST = "127.0.0.1"
CARLA_PORT = 2000
CARLA_TIMEOUT = 10.0


# ============================================================
# TEST AYARLARI
# ============================================================

CONTROL_PERIOD = 0.05
PRINT_INTERVAL = 1.0

INITIAL_HEADING_WARNING_DEG = 70.0
INITIAL_HEADING_ABORT_DEG = 120.0

WRONG_WAY_WARNING_DEG = 100.0


# ============================================================
# DEBUG AYARLARI
# ============================================================

# Statik rota çok uzun süre görünür kalsın.
STATIC_DEBUG_LIFETIME = 30.0

# Dinamik debug işaretleri her frame yeniden çizilecek.
DYNAMIC_DEBUG_LIFETIME = 0.12

ROUTE_LINE_THICKNESS = 0.09
ROUTE_POINT_SIZE = 0.07

TARGET_POINT_SIZE = 0.22
CURRENT_ROUTE_POINT_SIZE = 0.18


# ============================================================
# SPECTATOR KAMERA
# ============================================================

CAMERA_DISTANCE = 8.0
CAMERA_HEIGHT = 5.0
CAMERA_PITCH = -18.0


# ============================================================
# MAIN 9 - ADAPTIVE VEHICLE FOLLOWING
# ============================================================

FOLLOW_ACTIVATION_DISTANCE = 30.0
FOLLOW_FREE_DISTANCE = 26.0
FOLLOW_TARGET_DISTANCE = 18.0
FOLLOW_SLOW_DISTANCE = 13.0
FOLLOW_STOP_DISTANCE = 8.0
FOLLOW_LOCK_RELEASE_DISTANCE = 32.0
FOLLOW_REQUIRED_HITS = 2
FOLLOW_CLEAR_FRAMES = 8
LEAD_TARGET_SPEED_KMH = 7.0
LEAD_INITIAL_ROUTE_INDEX = 22
LEAD_Z_OFFSET = 0.55
VEHICLE_LABELS = {"car", "truck", "bus", "motorcycle"}

# ============================================================
# FINAL CAMPUS LIFE AYARLARI
# ============================================================

CAMPUS_NPC_VEHICLE_COUNT = 8
CAMPUS_PEDESTRIAN_COUNT = 10
CAMPUS_TRAFFIC_MANAGER_PORT = 8000
CAMPUS_MIN_SPAWN_DISTANCE_FROM_EGO = 12.0
CAMPUS_BAD_ZONE_AVOID_RADIUS = 14.0
CAMPUS_NPC_FRONT_LATERAL_LIMIT = 4.5
CAMPUS_NPC_FRONT_MAX_DISTANCE = 60.0

# ============================================================
# TEMEL YARDIMCI FONKSİYONLAR
# ============================================================

def distance_2d(location_a, location_b):

    dx = location_a.x - location_b.x
    dy = location_a.y - location_b.y

    return math.sqrt(
        dx * dx
        + dy * dy
    )


def normalize_angle_degrees(angle):

    while angle > 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return angle


def heading_between(location_a, location_b):

    dx = location_b.x - location_a.x
    dy = location_b.y - location_a.y

    return math.degrees(
        math.atan2(
            dy,
            dx,
        )
    )


# ============================================================
# SPECTATOR KAMERA
# ============================================================

def update_spectator_camera(
    world,
    vehicle,
):

    spectator = world.get_spectator()

    transform = vehicle.get_transform()

    vehicle_location = transform.location
    vehicle_yaw = transform.rotation.yaw

    yaw_rad = math.radians(
        vehicle_yaw
    )

    camera_location = carla.Location(
        x=(
            vehicle_location.x
            - CAMERA_DISTANCE
            * math.cos(yaw_rad)
        ),
        y=(
            vehicle_location.y
            - CAMERA_DISTANCE
            * math.sin(yaw_rad)
        ),
        z=(
            vehicle_location.z
            + CAMERA_HEIGHT
        ),
    )

    camera_rotation = carla.Rotation(
        pitch=CAMERA_PITCH,
        yaw=vehicle_yaw,
        roll=0.0,
    )

    spectator.set_transform(
        carla.Transform(
            camera_location,
            camera_rotation,
        )
    )


# ============================================================
# STATİK ROTA ÇİZİMİ
# ============================================================

def draw_static_route(
    world,
    route_locations,
):

    debug = world.debug

    print(
        "\n======================================"
    )

    print(
        "DEBUG ROTA ÇİZİLİYOR"
    )

    print(
        "======================================"
    )

    # --------------------------------------------------------
    # YEŞİL NOKTALAR
    # --------------------------------------------------------

    for location in route_locations:

        debug.draw_point(
            carla.Location(
                x=location.x,
                y=location.y,
                z=location.z + 0.35,
            ),
            size=ROUTE_POINT_SIZE,
            color=carla.Color(
                0,
                255,
                0,
            ),
            life_time=STATIC_DEBUG_LIFETIME,
            persistent_lines=False,
        )

    # --------------------------------------------------------
    # YEŞİL ROTA ÇİZGİSİ
    # --------------------------------------------------------

    for index in range(
        len(route_locations) - 1
    ):

        start = route_locations[index]
        end = route_locations[index + 1]

        debug.draw_line(
            carla.Location(
                x=start.x,
                y=start.y,
                z=start.z + 0.40,
            ),
            carla.Location(
                x=end.x,
                y=end.y,
                z=end.z + 0.40,
            ),
            thickness=ROUTE_LINE_THICKNESS,
            color=carla.Color(
                0,
                255,
                0,
            ),
            life_time=STATIC_DEBUG_LIFETIME,
            persistent_lines=False,
        )

    # --------------------------------------------------------
    # MAVİ YÖN OKLARI
    # --------------------------------------------------------

    arrow_step = 5

    for index in range(
        0,
        len(route_locations) - 1,
        arrow_step,
    ):

        end_index = min(
            index + 2,
            len(route_locations) - 1,
        )

        start = route_locations[index]
        end = route_locations[end_index]

        debug.draw_arrow(
            carla.Location(
                x=start.x,
                y=start.y,
                z=start.z + 0.75,
            ),
            carla.Location(
                x=end.x,
                y=end.y,
                z=end.z + 0.75,
            ),
            thickness=0.08,
            arrow_size=0.25,
            color=carla.Color(
                0,
                110,
                255,
            ),
            life_time=STATIC_DEBUG_LIFETIME,
            persistent_lines=False,
        )

    # --------------------------------------------------------
    # START
    # --------------------------------------------------------

    start = route_locations[0]

    debug.draw_point(
        carla.Location(
            x=start.x,
            y=start.y,
            z=start.z + 1.0,
        ),
        size=0.25,
        color=carla.Color(
            0,
            0,
            255,
        ),
        life_time=STATIC_DEBUG_LIFETIME,
        persistent_lines=False,
    )

    debug.draw_string(
        carla.Location(
            x=start.x,
            y=start.y,
            z=start.z + 1.8,
        ),
        "START",
        draw_shadow=True,
        color=carla.Color(
            0,
            0,
            255,
        ),
        life_time=STATIC_DEBUG_LIFETIME,
        persistent_lines=False,
    )

    # --------------------------------------------------------
    # GOAL
    # --------------------------------------------------------

    goal = route_locations[-1]

    debug.draw_point(
        carla.Location(
            x=goal.x,
            y=goal.y,
            z=goal.z + 1.0,
        ),
        size=0.28,
        color=carla.Color(
            255,
            0,
            0,
        ),
        life_time=STATIC_DEBUG_LIFETIME,
        persistent_lines=False,
    )

    debug.draw_string(
        carla.Location(
            x=goal.x,
            y=goal.y,
            z=goal.z + 1.8,
        ),
        "GOAL",
        draw_shadow=True,
        color=carla.Color(
            255,
            0,
            0,
        ),
        life_time=STATIC_DEBUG_LIFETIME,
        persistent_lines=False,
    )

    print(
        "YESIL  = A* rota"
    )

    print(
        "MAVI   = rota yonu"
    )

    print(
        "KIRMIZI = anlik Pure Pursuit hedefi"
    )

    print(
        "SARI   = arac -> hedef noktasi"
    )

    print(
        "CAMGOBEGI = mevcut route index"
    )


# ============================================================
# DİNAMİK PURE PURSUIT DEBUG
# ============================================================

def draw_dynamic_controller_debug(
    world,
    vehicle,
    route_locations,
    result,
):

    debug = world.debug

    route_index = result[
        "route_index"
    ]

    target_index = result[
        "target_index"
    ]

    route_index = max(
        0,
        min(
            route_index,
            len(route_locations) - 1,
        ),
    )

    target_index = max(
        0,
        min(
            target_index,
            len(route_locations) - 1,
        ),
    )

    current_route_location = (
        route_locations[
            route_index
        ]
    )

    target_location = (
        route_locations[
            target_index
        ]
    )

    vehicle_location = (
        vehicle.get_location()
    )

    # --------------------------------------------------------
    # CAMGÖBEĞİ:
    # Aracın mevcut route index'i
    # --------------------------------------------------------

    debug.draw_point(
        carla.Location(
            x=current_route_location.x,
            y=current_route_location.y,
            z=current_route_location.z + 0.85,
        ),
        size=CURRENT_ROUTE_POINT_SIZE,
        color=carla.Color(
            0,
            255,
            255,
        ),
        life_time=DYNAMIC_DEBUG_LIFETIME,
        persistent_lines=False,
    )

    # --------------------------------------------------------
    # KIRMIZI:
    # Pure Pursuit hedef noktası
    # --------------------------------------------------------

    debug.draw_point(
        carla.Location(
            x=target_location.x,
            y=target_location.y,
            z=target_location.z + 1.05,
        ),
        size=TARGET_POINT_SIZE,
        color=carla.Color(
            255,
            0,
            0,
        ),
        life_time=DYNAMIC_DEBUG_LIFETIME,
        persistent_lines=False,
    )

    # --------------------------------------------------------
    # SARI:
    # Araçtan Pure Pursuit hedef noktasına çizgi
    # --------------------------------------------------------

    debug.draw_line(
        carla.Location(
            x=vehicle_location.x,
            y=vehicle_location.y,
            z=vehicle_location.z + 1.30,
        ),
        carla.Location(
            x=target_location.x,
            y=target_location.y,
            z=target_location.z + 1.05,
        ),
        thickness=0.08,
        color=carla.Color(
            255,
            255,
            0,
        ),
        life_time=DYNAMIC_DEBUG_LIFETIME,
        persistent_lines=False,
    )

    # --------------------------------------------------------
    # TARGET INDEX YAZISI
    # --------------------------------------------------------

    debug.draw_string(
        carla.Location(
            x=target_location.x,
            y=target_location.y,
            z=target_location.z + 1.45,
        ),
        f"TARGET {target_index}",
        draw_shadow=True,
        color=carla.Color(
            255,
            0,
            0,
        ),
        life_time=DYNAMIC_DEBUG_LIFETIME,
        persistent_lines=False,
    )


# ============================================================
# ROTA YÖN KONTROLÜ
# ============================================================

def validate_route_direction(
    carla_map,
    route_locations,
):

    print(
        "\n=== ROTA YON KONTROLU ==="
    )

    suspicious_segments = []

    checked_segments = 0

    for index in range(
        len(route_locations) - 1
    ):

        current_location = (
            route_locations[index]
        )

        next_location = (
            route_locations[index + 1]
        )

        segment_distance = distance_2d(
            current_location,
            next_location,
        )

        if segment_distance < 0.20:
            continue

        waypoint = carla_map.get_waypoint(
            current_location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )

        if waypoint is None:
            continue

        route_heading = heading_between(
            current_location,
            next_location,
        )

        lane_heading = (
            waypoint.transform.rotation.yaw
        )

        heading_error = abs(
            normalize_angle_degrees(
                route_heading
                - lane_heading
            )
        )

        checked_segments += 1

        if (
            heading_error
            > WRONG_WAY_WARNING_DEG
        ):

            suspicious_segments.append(
                {
                    "index": index,
                    "error": heading_error,
                    "road_id": waypoint.road_id,
                    "lane_id": waypoint.lane_id,
                    "section_id": waypoint.section_id,
                }
            )

    print(
        "Kontrol edilen segment:",
        checked_segments,
    )

    print(
        "Supheli ters-yon segment:",
        len(suspicious_segments),
    )

    if not suspicious_segments:

        print(
            "Route direction check: OK"
        )

    else:

        print(
            "\nUYARI: Yon farki bulunan segmentler var."
        )

        for segment in suspicious_segments[:10]:

            print(
                "Index:",
                segment["index"],
                "| Error:",
                f'{segment["error"]:.1f}',
                "| Road:",
                segment["road_id"],
                "| Lane:",
                segment["lane_id"],
            )

    return suspicious_segments


# ============================================================
# BAŞLANGIÇ HEADING
# ============================================================

def check_initial_heading(
    vehicle,
    route_locations,
):

    if len(route_locations) < 2:
        return 0.0

    transform = (
        vehicle.get_transform()
    )

    vehicle_location = (
        transform.location
    )

    vehicle_yaw = (
        transform.rotation.yaw
    )

    nearest_index = 0
    nearest_distance = float("inf")

    search_limit = min(
        len(route_locations),
        30,
    )

    for index in range(
        search_limit
    ):

        distance = distance_2d(
            vehicle_location,
            route_locations[index],
        )

        if distance < nearest_distance:

            nearest_distance = distance
            nearest_index = index

    next_index = min(
        nearest_index + 2,
        len(route_locations) - 1,
    )

    route_heading = heading_between(
        route_locations[
            nearest_index
        ],
        route_locations[
            next_index
        ],
    )

    heading_error = (
        normalize_angle_degrees(
            route_heading
            - vehicle_yaw
        )
    )

    absolute_error = abs(
        heading_error
    )

    print(
        "\n=== BASLANGIC YON KONTROLU ==="
    )

    print(
        "Arac yaw:",
        f"{vehicle_yaw:.1f}"
    )

    print(
        "Rota yonu:",
        f"{route_heading:.1f}"
    )

    print(
        "Heading farki:",
        f"{heading_error:.1f}"
    )

    if (
        absolute_error
        <= INITIAL_HEADING_WARNING_DEG
    ):

        print(
            "Baslangic yonu: OK"
        )

    elif (
        absolute_error
        <= INITIAL_HEADING_ABORT_DEG
    ):

        print(
            "UYARI: Yuksek baslangic acisi."
        )

    else:

        print(
            "KRITIK: Arac ters yonde olabilir."
        )

    return absolute_error


# ============================================================
# ANA GIRIS - TAM ROTA KINEMATIK SURUS
# ============================================================

ANAGIRIS_KINEMATIC_SPEED_KMH = 7.0
ANAGIRIS_KINEMATIC_STEP = 0.02
ANAGIRIS_KINEMATIC_YAW_ALPHA = 0.25
ANAGIRIS_KINEMATIC_Z_OFFSET_MIN = -0.30
ANAGIRIS_KINEMATIC_Z_OFFSET_MAX = 0.20


def _angle_diff(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def _lerp_angle(current, target, alpha):
    return current + _angle_diff(target, current) * alpha


def _route_cumulative(route_locations):
    cumulative = [0.0]

    for i in range(1, len(route_locations)):
        cumulative.append(
            cumulative[-1]
            + distance_2d(
                route_locations[i - 1],
                route_locations[i],
            )
        )

    return cumulative


def _sample_route(route_locations, cumulative, distance):
    if distance <= 0.0:
        return route_locations[0], 0

    if distance >= cumulative[-1]:
        return route_locations[-1], len(route_locations) - 1

    hi = 1

    while (
        hi < len(cumulative)
        and cumulative[hi] < distance
    ):
        hi += 1

    lo = hi - 1

    segment_length = max(
        cumulative[hi] - cumulative[lo],
        1e-6,
    )

    ratio = (
        distance - cumulative[lo]
    ) / segment_length

    a = route_locations[lo]
    b = route_locations[hi]

    location = carla.Location(
        x=a.x + (b.x - a.x) * ratio,
        y=a.y + (b.y - a.y) * ratio,
        z=a.z + (b.z - a.z) * ratio,
    )

    return location, lo


# ============================================================
# MAIN 8 - YOLO YAYA GUVENLIK KATMANI
# ============================================================

class PedestrianSafety:
    """Ego icin world-coordinate yaya guvenligi.

    YOLO person kutulari goruntuleme/algilama icin kalir; fren karari yalnizca
    yayanin CARLA dunyasinda ego aracinin gelecekteki surus koridoruna gore
    verilir. Boylece kaldirim/cimdeki yakin bir yaya gereksiz STOP yaratmaz,
    fakat arac govdesine yakin veya yola dogru gecmekte olan yaya erken fark edilir.
    """

    CLEAR_FRAMES = 6
    # Yaklasik arac yari genisligi + guvenlik paylari (metre).
    STOP_LATERAL = 1.55
    DANGER_LATERAL = 2.20
    CAUTION_LATERAL = 2.85
    LOOKAHEAD = 22.0

    def __init__(self):
        self.clear_count = 0
        self.level = "CLEAR"
        self.last_reason = ""

    @staticmethod
    def _rank(level):
        return {"CLEAR":0, "WATCH":1, "CAUTION":2, "DANGER":3, "STOP":4}[level]

    def _world_level(self, vehicle, pedestrian_scenario):
        if vehicle is None or pedestrian_scenario is None:
            return "CLEAR", ""
        try:
            tf = vehicle.get_transform()
            loc = tf.location
            f = tf.get_forward_vector()
            r = tf.get_right_vector()
        except Exception:
            return "CLEAR", ""

        best_level = "CLEAR"
        best_reason = ""
        for walker in getattr(pedestrian_scenario, 'walkers', []):
            try:
                if walker.actor is None or not walker.actor.is_alive or walker.finished:
                    continue
                p = walker.actor.get_location()
                dx, dy = p.x-loc.x, p.y-loc.y
                forward = dx*f.x + dy*f.y
                lateral_signed = dx*r.x + dy*r.y
                lateral = abs(lateral_signed)

                # Arkamizdaki veya cok uzaktaki yaya fren sebebi degildir.
                if forward <= -1.5 or forward > self.LOOKAHEAD:
                    continue

                # Yayanin hedefi arac koridorunu kesiyor mu? Crossing yayasini
                # daha seride girmeden fark etmek icin mevcut + hedef lateral
                # konumunu birlikte kullan.
                approaching_path = False
                try:
                    t = walker.target
                    tdx, tdy = t.x-loc.x, t.y-loc.y
                    target_forward = tdx*f.x + tdy*f.y
                    target_lat_signed = tdx*r.x + tdy*r.y
                    crosses_center = lateral_signed * target_lat_signed <= 0.0
                    enters_corridor = abs(target_lat_signed) <= self.DANGER_LATERAL
                    approaching_path = (
                        max(forward, target_forward) > 0.0
                        and min(forward, target_forward) < self.LOOKAHEAD
                        and (crosses_center or enters_corridor)
                    )
                except Exception:
                    approaching_path = False

                level = "CLEAR"
                # Govde koridoru: yakin yaya gercek carpisma riski.
                if 0.0 < forward <= 6.0 and lateral <= self.STOP_LATERAL:
                    level = "STOP"
                elif 0.0 < forward <= 10.0 and lateral <= self.DANGER_LATERAL:
                    level = "DANGER"
                elif 0.0 < forward <= 16.0 and lateral <= self.CAUTION_LATERAL:
                    level = "CAUTION"
                # Yandan yola dogru gelen crossing: kenardayken erken yavasla,
                # fakat kaldirimda yola paralel yuruyen kisi icin durma.
                elif approaching_path and 0.0 < forward <= 10.0 and lateral <= 3.8:
                    level = "DANGER"
                elif approaching_path and 0.0 < forward <= 18.0 and lateral <= 4.5:
                    level = "CAUTION"
                elif 0.0 < forward <= self.LOOKAHEAD and lateral <= 4.5:
                    level = "WATCH"

                if self._rank(level) > self._rank(best_level):
                    best_level = level
                    best_reason = f"world fwd={forward:.1f}m lat={lateral:.1f}m crossing={approaching_path}"
            except Exception:
                pass
        return best_level, best_reason

    def update(self, vision_system=None, vehicle=None, pedestrian_scenario=None):
        # Fren karari world geometry'den gelir. YOLO bilerek bu karara dahil
        # edilmez; ekranda person kutusu gorunmesi tek basina fren demek degildir.
        new_level, reason = self._world_level(vehicle, pedestrian_scenario)

        # Tam durustan cikista kisa histerezis: yaya sinirdan cikar cikmaz
        # arac ileri atilmasin. Diger seviyeler anlik azalabilir.
        if self.level == "STOP" and new_level != "STOP":
            self.clear_count += 1
            if self.clear_count < self.CLEAR_FRAMES:
                new_level = "STOP"
            else:
                self.clear_count = 0
        else:
            self.clear_count = 0

        if new_level != self.level:
            if new_level == "STOP":
                print("\n>>> YAYA ARAC KORIDORUNDA - TAM DURUS <<<")
            elif new_level == "DANGER":
                print("\n>>> YAYA CARPISMA KORIDORUNA YAKIN - 6 km/h <<<")
            elif new_level == "CAUTION":
                print("\n>>> YAYA YOLA YAKLASIYOR - 10 km/h <<<")
            elif new_level in ("WATCH", "CLEAR") and self.level in ("STOP", "DANGER", "CAUTION"):
                print("\n>>> YAYA KORIDORU TEMIZ - NORMAL SURUSE DONUS <<<")
        self.level = new_level
        self.last_reason = reason
        return self.level

    def speed_limit(self, base_speed_kmh):
        if self.level == "STOP":
            return 0.0
        if self.level == "DANGER":
            return min(base_speed_kmh, 6.0)
        if self.level == "CAUTION":
            return min(base_speed_kmh, 10.0)
        return base_speed_kmh


# ============================================================
# MAIN 8 - KONTROLLU YAYA KARSIDAN KARSIYA GECIS SENARYOSU
# ============================================================

class PedestrianCrossingScenario:
    """Eski calisan yaya mantiginin Main 8'e tasinmis hali.

    Main 8'in rota/YOLO/fren/collision-safe yapisina dokunmaz.
    Sadece test yayasinin spawn ve WalkerControl davranisini eski calisan
    surumdeki prensiple uygular.
    """

    TRIGGER_DISTANCE = 25.0
    START_LATERAL = 4.2
    WALK_DISTANCE = 10.0
    WALK_SPEED = 1.5
    FINISH_TOLERANCE = 0.8
    Z_OFFSET = 0.35

    def __init__(self, world, route_locations):
        self.world = world
        self.route_locations = route_locations
        self.walker = None
        self.started = False
        self.finished = False
        self.removed = False
        self.route_index = 0
        self.destination = None
        self.walk_direction = None
        self._spawn()

    def _route_direction(self, route_index):
        route_index = min(max(route_index, 0), len(self.route_locations) - 2)
        a = self.route_locations[route_index]
        b = self.route_locations[min(route_index + 5, len(self.route_locations) - 1)]
        dx, dy = b.x - a.x, b.y - a.y
        length = max(math.sqrt(dx * dx + dy * dy), 0.001)
        return carla.Vector3D(x=dx / length, y=dy / length, z=0.0)

    def _spawn(self):
        if len(self.route_locations) < 20:
            print('Yaya senaryosu: rota cok kisa, test yayasi olusturulmadi.')
            return

        # Duraklardan bagimsiz: secilen rotanin yaklasik %35 noktasini kullan.
        self.route_index = max(
            5,
            min(len(self.route_locations) - 10, int(len(self.route_locations) * 0.35)),
        )

        route_location = self.route_locations[self.route_index]
        route_direction = self._route_direction(self.route_index)

        # ESKI CALISAN KODLA AYNI SAG VE GECIS VEKTORU HESABI.
        right = carla.Vector3D(
            x=route_direction.y,
            y=-route_direction.x,
            z=0.0,
        )
        side = 1.0

        start_location = carla.Location(
            x=route_location.x + right.x * self.START_LATERAL * side,
            y=route_location.y + right.y * self.START_LATERAL * side,
            z=route_location.z + self.Z_OFFSET,
        )

        self.walk_direction = carla.Vector3D(
            x=-right.x * side,
            y=-right.y * side,
            z=0.0,
        )

        self.destination = carla.Location(
            x=start_location.x + self.walk_direction.x * self.WALK_DISTANCE,
            y=start_location.y + self.walk_direction.y * self.WALK_DISTANCE,
            z=start_location.z,
        )

        yaw = math.degrees(math.atan2(self.walk_direction.y, self.walk_direction.x))
        transform = carla.Transform(start_location, carla.Rotation(yaw=yaw))

        walkers = self.world.get_blueprint_library().filter('walker.pedestrian.*')
        if not walkers:
            print('Yaya senaryosu: walker blueprint bulunamadi.')
            return

        bp = walkers[0]
        if bp.has_attribute('is_invincible'):
            bp.set_attribute('is_invincible', 'false')

        self.walker = self.world.try_spawn_actor(bp, transform)
        if self.walker is None:
            print('Yaya senaryosu: test yayasi spawn edilemedi.')
            return

        # Eski kod gibi spawn olur olmaz sifir WalkerControl ile beklet.
        self.walker.apply_control(
            carla.WalkerControl(
                direction=carla.Vector3D(x=0.0, y=0.0, z=0.0),
                speed=0.0,
                jump=False,
            )
        )

        print('\n======================================')
        print('MAIN 8 - ESKI CALISAN YAYA MANTIGI AKTIF')
        print('======================================')
        print(f'Route index: {self.route_index}/{len(self.route_locations)-1}')
        print(f'X={start_location.x:.2f} Y={start_location.y:.2f} Z={start_location.z:.2f}')
        print(f'Arac {self.TRIGGER_DISTANCE:.0f} m yaklasinca yaya yurume kontrolu alacak.')

    def _reached_destination(self):
        if self.walker is None or not self.walker.is_alive:
            return True
        return distance_2d(self.walker.get_location(), self.destination) <= self.FINISH_TOLERANCE

    def update(self, vehicle):
        if self.walker is None or self.finished or self.removed:
            return
        if not self.walker.is_alive:
            self.finished = True
            return

        vehicle_location = vehicle.get_location()
        actor_distance = distance_2d(vehicle_location, self.walker.get_location())

        # ESKI CALISAN KOD: trigger gelince WalkerControl BIR KEZ baslatilir.
        if not self.started and actor_distance <= self.TRIGGER_DISTANCE:
            self.walker.apply_control(
                carla.WalkerControl(
                    direction=self.walk_direction,
                    speed=self.WALK_SPEED,
                    jump=False,
                )
            )
            self.started = True
            print('\n>>> YAYA KARSIYA GECMEYE BASLADI - ESKI CALISAN WALKERCONTROL <<<')

        if self.started and not self.finished and self._reached_destination():
            self.finished = True
            self.walker.apply_control(
                carla.WalkerControl(
                    direction=carla.Vector3D(x=0.0, y=0.0, z=0.0),
                    speed=0.0,
                    jump=False,
                )
            )
            print('\n>>> YAYA KARSIYA GECISI TAMAMLADI <<<')

    def destroy(self):
        if self.walker is not None:
            try:
                if self.walker.is_alive:
                    self.walker.destroy()
            except Exception:
                pass
            self.walker = None
            print('Test yayasi temizlendi.')


# ============================================================
# MAIN 9 - HAREKETLI LEAD VEHICLE + YOLO TAKIP KARARI
# ============================================================

class LeadVehicleScenario:
    """Test lead vehicle'i mevcut A* rotasi uzerinde kinematik ilerletir.

    Ego route/controller koduna dokunmaz. Lead actor gercek CARLA vehicle
    oldugu icin on RGB kamera + YOLO tarafindan gorulebilir.
    """

    def __init__(self, world, route_locations):
        self.world = world
        self.route_locations = route_locations
        self.actor = None
        self.cumulative = _route_cumulative(route_locations)
        self.total_distance = self.cumulative[-1]
        self.travelled = 0.0
        self.last_update = time.perf_counter()
        self._spawn()

    def _choose_blueprint(self):
        library = self.world.get_blueprint_library()
        for blueprint_id in (
            "vehicle.audi.tt",
            "vehicle.tesla.model3",
            "vehicle.lincoln.mkz_2020",
        ):
            matches = library.filter(blueprint_id)
            if matches:
                return matches[0]
        vehicles = library.filter("vehicle.*")
        if not vehicles:
            raise RuntimeError("Lead vehicle blueprint bulunamadi.")
        return vehicles[0]

    def _spawn(self):
        if len(self.route_locations) < 12:
            raise RuntimeError("Lead vehicle icin rota cok kisa.")

        candidates = [
            min(LEAD_INITIAL_ROUTE_INDEX, len(self.route_locations) - 8),
            min(LEAD_INITIAL_ROUTE_INDEX + 6, len(self.route_locations) - 8),
            min(LEAD_INITIAL_ROUTE_INDEX + 12, len(self.route_locations) - 8),
        ]
        candidates = list(dict.fromkeys(max(5, i) for i in candidates))
        blueprint = self._choose_blueprint()

        for route_index in candidates:
            location = self.route_locations[route_index]
            ahead = self.route_locations[min(route_index + 4, len(self.route_locations) - 1)]
            yaw = heading_between(location, ahead)
            for z_offset in (LEAD_Z_OFFSET, 0.8, 1.0, 1.2):
                actor = self.world.try_spawn_actor(
                    blueprint,
                    carla.Transform(
                        carla.Location(x=location.x, y=location.y, z=location.z + z_offset),
                        carla.Rotation(yaw=yaw),
                    ),
                )
                if actor is None:
                    continue
                self.actor = actor
                self.actor.set_autopilot(False)
                self.actor.set_simulate_physics(False)
                self.actor.set_enable_gravity(False)
                self.travelled = self.cumulative[route_index]
                self.last_update = time.perf_counter()
                print(
                    f"\nMAIN9 lead vehicle spawn OK | Route: {route_index} | "
                    f"Hiz: {LEAD_TARGET_SPEED_KMH:.1f} km/h"
                )
                return

        raise RuntimeError("Lead vehicle spawn edilemedi.")

    def update(self):
        if self.actor is None or not self.actor.is_alive:
            return

        now = time.perf_counter()
        dt = max(0.0, min(now - self.last_update, 0.15))
        self.last_update = now
        self.travelled = min(
            self.total_distance,
            self.travelled + (LEAD_TARGET_SPEED_KMH / 3.6) * dt,
        )

        location, _ = _sample_route(
            self.route_locations, self.cumulative, self.travelled
        )
        ahead, _ = _sample_route(
            self.route_locations,
            self.cumulative,
            min(self.total_distance, self.travelled + 2.0),
        )
        yaw = heading_between(location, ahead)
        current = self.actor.get_transform()
        z_delta = current.location.z - location.z
        z_delta = max(0.20, min(1.20, z_delta))
        self.actor.set_transform(
            carla.Transform(
                carla.Location(x=location.x, y=location.y, z=location.z + z_delta),
                carla.Rotation(yaw=yaw),
            )
        )

    def distance_to(self, ego_vehicle):
        if self.actor is None or not self.actor.is_alive:
            return float("inf")
        return distance_2d(ego_vehicle.get_location(), self.actor.get_location())

    def destroy(self):
        if self.actor is not None:
            try:
                if self.actor.is_alive:
                    self.actor.destroy()
            except Exception:
                pass
            self.actor = None


class VehicleFollowingSafety:
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 360
    MIN_CONFIDENCE = 0.18
    ROI_X_MIN = 0.25
    ROI_X_MAX = 0.75
    ROI_BOTTOM_MIN = 0.40
    MIN_BOX_HEIGHT_RATIO = 0.08

    def __init__(self):
        self.hit_count = 0
        self.clear_count = 0
        self.locked = False
        self.stop_latched = False
        self.last_state = "SERBEST SURUS"

    def _front_vehicle(self, detection):
        label = str(detection.get("label", "")).lower()
        if label not in VEHICLE_LABELS:
            return False
        if float(detection.get("confidence", 0.0)) < self.MIN_CONFIDENCE:
            return False
        bbox = detection.get("bbox")
        if not bbox or len(bbox) != 4:
            return False
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) * 0.5
        box_h = max(0.0, float(y2 - y1))
        return (
            self.ROI_X_MIN <= cx / self.FRAME_WIDTH <= self.ROI_X_MAX
            and float(y2) / self.FRAME_HEIGHT >= self.ROI_BOTTOM_MIN
            and box_h / self.FRAME_HEIGHT >= self.MIN_BOX_HEIGHT_RATIO
        )

    def update(self, vision_system, lead_distance):
        detections = []
        if vision_system is not None:
            try:
                detections = vision_system.get_detections() or []
            except Exception:
                detections = []

        seen = any(self._front_vehicle(d) for d in detections)
        if seen:
            self.hit_count += 1
            self.clear_count = 0
        else:
            self.hit_count = 0
            self.clear_count += 1

        if self.hit_count >= FOLLOW_REQUIRED_HITS and lead_distance <= FOLLOW_ACTIVATION_DISTANCE:
            if not self.locked:
                print(">>> YOLO ON ARAC DOGRULANDI - ADAPTIVE FOLLOW AKTIF <<<")
            self.locked = True

        # Guvenlik fallback: gercek test lead'i kritik mesafeye girdiyse
        # YOLO'nun tek kare kacirmasi ego'nun carpmasina izin vermesin.
        if lead_distance <= FOLLOW_STOP_DISTANCE:
            self.locked = True

        if (
            self.locked
            and lead_distance >= FOLLOW_LOCK_RELEASE_DISTANCE
            and self.clear_count >= FOLLOW_CLEAR_FRAMES
        ):
            self.locked = False
            print(">>> ON ARAC TAKIP BOLGESINDEN CIKTI - SERBEST SURUS <<<")

        return self.locked, seen

    def speed_limit(self, route_speed, lead_distance, lead_speed=LEAD_TARGET_SPEED_KMH):
        # V8 FOLLOW HYSTERESIS:
        # 8.0 m veya altinda bir kez durduysak, mesafe 10.5 m'ye acilmadan
        # tekrar hareket etmiyoruz. Boylece 7.9 <-> 8.1 m civarindaki
        # DUR / YAVASLA salinimi engellenir.
        stop_release_distance = 10.5

        if not self.locked or lead_distance > FOLLOW_FREE_DISTANCE:
            self.stop_latched = False
            state = "SERBEST SURUS"
            limit = route_speed
        else:
            if lead_distance <= FOLLOW_STOP_DISTANCE:
                self.stop_latched = True
            elif self.stop_latched and lead_distance >= stop_release_distance:
                self.stop_latched = False

            if self.stop_latched:
                state = "ARAC ICIN DUR"
                limit = 0.0
            elif lead_distance <= FOLLOW_SLOW_DISTANCE:
                slow_span = max(0.1, FOLLOW_SLOW_DISTANCE - FOLLOW_STOP_DISTANCE)
                slow_ratio = (lead_distance - FOLLOW_STOP_DISTANCE) / slow_span
                smooth_slow_limit = 3.0 + 3.0 * max(0.0, min(1.0, slow_ratio))
                state = "ARAC ICIN YAVASLA"
                limit = min(route_speed, smooth_slow_limit)
            else:
                distance_error = lead_distance - FOLLOW_TARGET_DISTANCE
                adaptive_speed = lead_speed + 0.35 * distance_error
                limit = max(4.0, min(route_speed, adaptive_speed))
                state = "ADAPTIVE FOLLOW"

        if state != self.last_state:
            print(f">>> MAIN10: {state} | Mesafe: {lead_distance:.1f} m | Limit: {limit:.1f} km/h <<<")
            self.last_state = state
        return limit, state



# ============================================================
# MAIN 10 - COKLU NESNE SENARYOSU
# ============================================================

class MultiVehicleScenario:
    """Birden fazla gercek CARLA vehicle actor'u ayni A* rota uzerinde ilerletir.

    Main9'daki LeadVehicleScenario degistirilmez. Her arac rotanin farkli bir
    diliminde baslatilir. Ego icin kritik mesafe, yasayan test araclari
    arasindaki EN YAKIN mesafedir.
    """

    def __init__(self, world, route_locations):
        self.world = world
        self.route_locations = route_locations
        self.scenarios = []

        # Birinci arac Main9 ile ayni yerde.
        route_slices = [route_locations]

        # Ikinci araci daha ileride baslat. Rota cok kisaysa ekleme.
        second_offset = 24
        if len(route_locations) - second_offset >= 35:
            route_slices.append(route_locations[second_offset:])

        # Ucuncu arac icin yeterli uzunluk varsa daha ileride bir tane daha.
        third_offset = 52
        if len(route_locations) - third_offset >= 35:
            route_slices.append(route_locations[third_offset:])

        for number, route_slice in enumerate(route_slices, start=1):
            try:
                scenario = LeadVehicleScenario(world, route_slice)
                self.scenarios.append(scenario)
                print(
                    f">>> MAIN10 TEST ARACI {number} HAZIR | "
                    f"Aktif arac: {len(self.scenarios)} <<<"
                )
            except Exception as exc:
                print(
                    f">>> MAIN10 TEST ARACI {number} OLUSTURULAMADI: {exc} <<<"
                )

        if not self.scenarios:
            raise RuntimeError("MAIN10: hicbir test araci spawn edilemedi.")

    def update(self):
        for scenario in self.scenarios:
            scenario.update()

    def distance_to(self, ego_vehicle):
        distances = []
        for scenario in self.scenarios:
            try:
                d = scenario.distance_to(ego_vehicle)
                if math.isfinite(d):
                    distances.append(d)
            except Exception:
                pass
        return min(distances) if distances else float("inf")

    def active_count(self):
        count = 0
        for scenario in self.scenarios:
            actor = getattr(scenario, "actor", None)
            if actor is not None:
                try:
                    if actor.is_alive:
                        count += 1
                except Exception:
                    pass
        return count

    def destroy(self):
        for scenario in self.scenarios:
            try:
                scenario.destroy()
            except Exception:
                pass
        self.scenarios.clear()


class MultiPedestrianScenario:
    """Birden fazla yaya crossing denemesi.

    Main8'deki PedestrianCrossingScenario yeniden yazilmaz. Farkli rota
    dilimleri verilerek farkli noktalarda iki yaya olusturulmaya calisilir.
    Walker spawn basarisiz olsa bile Main10 ana surus sistemi devam eder.
    """

    def __init__(self, world, route_locations):
        self.scenarios = []

        slices = [route_locations]

        offset = max(20, int(len(route_locations) * 0.28))
        if len(route_locations) - offset >= 30:
            slices.append(route_locations[offset:])

        for number, route_slice in enumerate(slices, start=1):
            try:
                scenario = PedestrianCrossingScenario(world, route_slice)
                if scenario.walker is not None:
                    self.scenarios.append(scenario)
                    print(
                        f">>> MAIN10 TEST YAYASI {number} HAZIR | "
                        f"Aktif yaya: {len(self.scenarios)} <<<"
                    )
                else:
                    print(
                        f">>> MAIN10 TEST YAYASI {number} SPAWN OLMADI - "
                        "ANA SISTEM DEVAM EDIYOR <<<"
                    )
            except Exception as exc:
                print(
                    f">>> MAIN10 TEST YAYASI {number} HATASI: {exc} - "
                    "ANA SISTEM DEVAM EDIYOR <<<"
                )

    def update(self, vehicle):
        for scenario in self.scenarios:
            try:
                scenario.update(vehicle)
            except Exception:
                pass

    def active_count(self):
        count = 0
        for scenario in self.scenarios:
            walker = getattr(scenario, "walker", None)
            if walker is not None:
                try:
                    if walker.is_alive and not scenario.finished:
                        count += 1
                except Exception:
                    pass
        return count

    def destroy(self):
        for scenario in self.scenarios:
            try:
                scenario.destroy()
            except Exception:
                pass
        self.scenarios.clear()


# ============================================================
# FINAL CAMPUS LIFE - BAGIMSIZ NPC TRAFIK + COKLU YAYA
# ============================================================

class CampusTrafficScenario:
    """NPC'leri CARLA driving waypoint zincirinde surer.

    V5 farki: NPC'lerin bir kismi ego rotasi uzerinde/cevresinde bilincli olarak
    yerlestirilir; boylece terminalde 8/8 yazip araclar kampusun baska ucunda
    kalmaz. Kalan NPC'ler tum kampus waypoint havuzundan secilir. Ego'nun
    controller/collision mantigina dokunulmaz.
    """
    def __init__(self, client, world, ego_vehicle, count=CAMPUS_NPC_VEHICLE_COUNT, ego_route=None):
        self.client=client; self.world=world; self.ego_vehicle=ego_vehicle
        self.map=world.get_map(); self.actors=[]; self.states={}
        self.ego_route = ego_route or []
        self.debug_timer = 0.0
        self.last_update = time.perf_counter()
        self.pedestrian_scenario = None
        self._spawn(count)

    def _vehicle_blueprints(self):
        out=[]
        for bp in self.world.get_blueprint_library().filter('vehicle.*'):
            try:
                if bp.has_attribute('number_of_wheels') and int(bp.get_attribute('number_of_wheels')) != 4:
                    continue
            except Exception: pass
            if 'sprinter' not in bp.id.lower(): out.append(bp)
        return out

    def _spawn_one(self, wp, bps, ego, bad):
        try:
            loc = wp.transform.location
            if distance_2d(loc, ego) < 22.0:
                return False
            if distance_2d(loc, bad) < CAMPUS_BAD_ZONE_AVOID_RADIUS:
                return False
            if any(distance_2d(loc, a.get_location()) < 12.0 for a in self.actors if a.is_alive):
                return False

            bp = random.choice(bps)
            if bp.has_attribute('role_name'):
                bp.set_attribute('role_name', 'campus_npc')
            tf = wp.transform
            actor = self.world.try_spawn_actor(
                bp,
                carla.Transform(
                    carla.Location(tf.location.x, tf.location.y, tf.location.z + 0.35),
                    tf.rotation,
                ),
            )
            if actor is None:
                return False

            # NPC hareketi set_transform ile yapiliyor; harita fizik hatalarindan
            # ve Traffic Manager/nav sorunlarindan etkilenmez.
            actor.set_simulate_physics(False)
            actor.set_enable_gravity(False)
            self.actors.append(actor)
            initial_speed = random.uniform(4.5, 5.5)  # V7: ~16.2-19.8 km/h
            self.states[actor.id] = {
                'wp': wp,
                'speed': initial_speed,
                'target_speed': random.uniform(5.0, 6.1),  # V7: ~18.0-22.0 km/h
                'speed_change_timer': 0.0,
                'next_speed_change': random.uniform(4.0, 8.0),
                'progress': 0.0,
            }
            return True
        except Exception:
            return False

    def _spawn(self, requested):
        bps = self._vehicle_blueprints()
        if not bps:
            print('CAMPUS LIFE V7: NPC vehicle blueprint bulunamadi.')
            return

        ego = self.ego_vehicle.get_location()
        bad = carla.Location(x=347.0, y=-911.75, z=0.0)

        # 1) GORUNURLUK GARANTISI: once ego rotasinin ilerleyen kisimlari.
        # Ego'nun hemen onune yigma yapmiyoruz; farkli bolgelere dagitiyoruz.
        route_candidates = []
        if self.ego_route and len(self.ego_route) > 40:
            fractions = (0.10, 0.20, 0.32, 0.45, 0.58, 0.72)
            for frac in fractions:
                idx = min(len(self.ego_route) - 2, max(12, int((len(self.ego_route)-1) * frac)))
                try:
                    wp = self.map.get_waypoint(
                        self.ego_route[idx],
                        project_to_road=True,
                        lane_type=carla.LaneType.Driving,
                    )
                    if wp is not None:
                        route_candidates.append(wp)
                except Exception:
                    pass

        visible_target = min(requested, max(3, requested // 2))
        for wp in route_candidates:
            if len(self.actors) >= visible_target:
                break
            self._spawn_one(wp, bps, ego, bad)

        # 2) Kalan trafik kampusun genel yol agina dagilir.
        try:
            candidates = list(self.map.generate_waypoints(10.0))
        except Exception:
            candidates = []
        random.shuffle(candidates)
        for wp in candidates:
            if len(self.actors) >= requested:
                break
            self._spawn_one(wp, bps, ego, bad)

        print(
            f'>>> CAMPUS LIFE V7: {len(self.actors)}/{requested} HAREKETLI NPC ARAC AKTIF '
            f'(en az {min(len(self.actors), visible_target)} tanesi ego rotasi yakininda) <<<'
        )

    def _next(self,wp,d=2.0):
        try: opts=list(wp.next(d))
        except Exception: opts=[]
        return random.choice(opts) if opts else None

    def _blocked(self,actor):
        try:
            tf=actor.get_transform(); loc=tf.location; f=tf.get_forward_vector()
            # Yalnizca AYNI YONDE, NPC'nin gercekten onunde bulunan arac bloklar.
            # Karsi seritten gelen / ters yone giden NPC bu kontrole girmez.
            for other in self.actors:
                if other is actor or other is None or not other.is_alive: continue
                otf = other.get_transform()
                of = otf.get_forward_vector()
                heading_dot = f.x*of.x + f.y*of.y
                if heading_dot <= 0.35:
                    continue
                o=otf.location; dx=o.x-loc.x; dy=o.y-loc.y
                forward=dx*f.x+dy*f.y; lateral=abs(dx*(-f.y)+dy*f.x)
                if 0<forward<8 and lateral<2.2:
                    return True
        except Exception: pass
        return False

    def _npc_conflict_speed_limit(self, actor, current_speed):
        """NPC-NPC guvenligi: ayni serit takibinden ayri kavsak/capraz trafik kontrolu.

        Karsi seritte ters yonde giden araclar burada bilerek yok sayilir.
        Gercek capraz rota riski varsa deterministik oncelik uygulanir: actor ID'si
        kucuk olan gecer, buyuk olan yavaslar/durur. Boylece iki arac ayni anda
        birbirini bekleyip deadlock olusturmaz; gecen arac uzaklasinca limit kalkar.
        """
        try:
            tf = actor.get_transform()
            loc = tf.location
            f = tf.get_forward_vector()

            for other in self.actors:
                if other is actor or other is None or not other.is_alive:
                    continue

                otf = other.get_transform()
                oloc = otf.location
                of = otf.get_forward_vector()
                dx, dy = oloc.x - loc.x, oloc.y - loc.y
                dist = math.hypot(dx, dy)
                if dist >= 26.0:
                    continue

                signed_dot = f.x * of.x + f.y * of.y

                # Paralel ayni yon _blocked() tarafindan yonetilir.
                # Ters yon/karsi serit de kavsak riski sayilmaz.
                # Sadece belirgin capraz acili araclar bu bolume girer.
                if abs(signed_dot) >= 0.82:
                    continue

                other_state = self.states.get(other.id, {})
                other_speed = float(other_state.get('speed', 0.0))

                # Goreli hareketle 3.5 saniyelik closest-approach tahmini.
                rvx = of.x * other_speed - f.x * current_speed
                rvy = of.y * other_speed - f.y * current_speed
                vv = rvx * rvx + rvy * rvy
                if vv <= 0.20:
                    continue

                tca = -(dx * rvx + dy * rvy) / vv
                if not (0.0 < tca < 3.5):
                    continue

                cdx = dx + rvx * tca
                cdy = dy + rvy * tca
                closest = math.hypot(cdx, cdy)
                if closest >= 4.0:
                    continue

                # Tek taraf yol verir: kucuk actor ID onceliklidir.
                # Boylece iki NPC'nin sonsuza kadar birbirini beklemesi engellenir.
                if actor.id < other.id:
                    continue

                if dist <= 7.0 or tca <= 1.4:
                    return 0.0, f'NPC KAVSAK {other.id} - DUR'
                if tca <= 2.4:
                    return 1.6, f'NPC KAVSAK {other.id} - YAVAS'
                return 3.2, f'NPC KAVSAK {other.id} - YAKLASIYOR'

            return None, 'NPC SERBEST'
        except Exception:
            return None, 'NPC SERBEST'

    def _ego_speed_limit(self, actor, current_speed):
        """NPC'nin ego araca carpmasini onlemek icin kademeli hiz limiti.

        1) Ego NPC'nin on koridorundaysa mesafeye gore yavasla/dur.
        2) Kavsakta yollar kesisecekse 3 saniyelik closest-approach tahminiyle
           NPC'yi onceden yavaslat/durdur. Ego controller'a dokunulmaz.
        """
        try:
            ego = self.ego_vehicle
            if ego is None or not ego.is_alive:
                return None, 'EGO YOK'

            tf = actor.get_transform()
            loc = tf.location
            f = tf.get_forward_vector()
            r = tf.get_right_vector()
            e = ego.get_location()
            dx, dy = e.x - loc.x, e.y - loc.y
            dist = math.hypot(dx, dy)
            forward = dx * f.x + dy * f.y
            lateral = abs(dx * r.x + dy * r.y)

            ego_tf = ego.get_transform()
            ef = ego_tf.get_forward_vector()
            signed_heading_dot = f.x * ef.x + f.y * ef.y

            # Ayni/benzer seritte ego NPC'nin onundaysa adaptive-follow.
            # Karsi seritte ters yone giden ego/NPC birbirini takip araci sanmaz.
            if signed_heading_dot > 0.35 and 0.0 < forward < 24.0 and lateral < 4.0:
                if forward <= 6.5:
                    return 0.0, 'EGO COK YAKIN - DUR'
                if forward <= 10.0:
                    return 1.6, 'EGO TEHLIKELI - YAVAS'      # ~5.8 km/h
                if forward <= 16.0:
                    return 3.2, 'EGO YAKIN - YAVAS'          # ~11.5 km/h

            # Kavsak / capraz trafik: iki aracin mevcut yon ve hizlariyla
            # yakin gelecekteki en yakin yaklasma mesafesini tahmin et.
            if dist < 28.0:
                heading_dot = abs(signed_heading_dot)

                # Yalnizca yonler belirgin bicimde farkliysa kavsak kontrolu.
                if heading_dot < 0.88:
                    ev = ego.get_velocity()
                    rvx = ev.x - f.x * current_speed
                    rvy = ev.y - f.y * current_speed
                    vv = rvx * rvx + rvy * rvy
                    if vv > 0.20:
                        tca = -(dx * rvx + dy * rvy) / vv
                        if 0.0 < tca < 3.5:
                            cdx = dx + rvx * tca
                            cdy = dy + rvy * tca
                            closest = math.hypot(cdx, cdy)
                            if closest < 3.6:
                                if tca <= 1.5 or dist <= 7.0:
                                    return 0.0, 'KAVSAK EGO RISKI - DUR'
                                if tca <= 2.4:
                                    return 1.6, 'KAVSAK EGO RISKI - YAVAS'
                                return 3.2, 'KAVSAK EGO YAKLASIYOR'

            return None, 'SERBEST'
        except Exception:
            return None, 'SERBEST'

    def set_pedestrian_scenario(self, pedestrian_scenario):
        self.pedestrian_scenario = pedestrian_scenario

    def _pedestrian_speed_limit(self, actor):
        """NPC icin yaya mesafesine gore kademeli hiz limiti.

        Yaya uzaktaysa normal trafik; yakinda yavas; cok yakinda tam durus.
        Sadece NPC'nin ileri koridorundaki aktif yayalar hesaba katilir.
        """
        scenario = self.pedestrian_scenario
        if scenario is None:
            return None, 'YAYA YOK'
        try:
            tf = actor.get_transform()
            loc = tf.location
            f = tf.get_forward_vector()
            r = tf.get_right_vector()
        except Exception:
            return None, 'YAYA YOK'

        nearest = float('inf')
        for walker in getattr(scenario, 'walkers', []):
            try:
                if walker.actor is None or not walker.actor.is_alive or walker.finished:
                    continue
                p = walker.actor.get_location()
                dx, dy = p.x - loc.x, p.y - loc.y
                forward = dx * f.x + dy * f.y
                lateral = abs(dx * r.x + dy * r.y)
                # Genis erken algilama, fakat yan kaldirimdaki yayaya gereksiz
                # tam fren yok. Arac yoluna yaklastikca limit sertlesir.
                if 0.0 < forward < 20.0 and lateral < 3.2:
                    nearest = min(nearest, forward)
            except Exception:
                pass

        if not math.isfinite(nearest):
            return None, 'YAYA YOK'
        if nearest <= 6.0:
            return 0.0, 'YAYA COK YAKIN - DUR'
        if nearest <= 10.0:
            return 1.7, 'YAYA TEHLIKELI - YAVAS'   # ~6.1 km/h
        if nearest <= 15.0:
            return 3.0, 'YAYA YAKIN - YAVAS'       # ~10.8 km/h
        return None, 'YAYA IZLE'

    def update(self):
        now = time.perf_counter()
        dt = max(0.01, min(now - self.last_update, 0.15))
        self.last_update = now
        self.debug_timer += dt
        print_npc_debug = self.debug_timer >= 1.0
        if print_npc_debug:
            self.debug_timer = 0.0
        npc_debug = []

        for actor in self.actors:
            if actor is None or not actor.is_alive:
                continue
            s=self.states.get(actor.id)
            if not s:
                continue

            if print_npc_debug:
                npc_debug.append(f"{actor.id}:{s['speed']*3.6:.1f}/{s['target_speed']*3.6:.1f}")

            # V6: NPC'ler sabit hizla gitmez. 4-8 saniyede bir yeni bir
            # dogal hedef hiz secer ve mevcut hiz bu hedefe yumusak yaklasir.
            s['speed_change_timer'] += dt
            if s['speed_change_timer'] >= s['next_speed_change']:
                s['target_speed'] = random.uniform(5.0, 6.1)  # V7: 18.0-22.0 km/h
                s['speed_change_timer'] = 0.0
                s['next_speed_change'] = random.uniform(4.0, 8.0)

            blocked = self._blocked(actor)
            pedestrian_limit, pedestrian_state = self._pedestrian_speed_limit(actor)
            ego_limit, ego_state = self._ego_speed_limit(actor, s['speed'])
            npc_conflict_limit, npc_conflict_state = self._npc_conflict_speed_limit(actor, s['speed'])

            desired_speed = s['target_speed']
            if blocked:
                desired_speed = 0.0
            if pedestrian_limit is not None:
                desired_speed = min(desired_speed, pedestrian_limit)
            if ego_limit is not None:
                desired_speed = min(desired_speed, ego_limit)
            if npc_conflict_limit is not None:
                desired_speed = min(desired_speed, npc_conflict_limit)

            # Yaya cok yakinsa kinematik NPC aninda tutulur; daha uzakta
            # kademeli fren uygulanir.
            if pedestrian_limit == 0.0:
                s['speed'] = 0.0

            # Ego cok yakin/carpisma tahmini cok kritikse kinematik NPC'yi
            # aninda tut. Daha uzakta ise asagidaki decel ile kademeli yavaslar.
            if ego_limit == 0.0:
                s['speed'] = 0.0

            # NPC-NPC kavsak riskinde yol veren arac kritik mesafede aninda tutulur.
            # Oncelikli arac devam ettigi icin iki tarafli sonsuz bekleme olusmaz.
            if npc_conflict_limit == 0.0:
                s['speed'] = 0.0

            # Normal hiz korunur; sadece riskte daha guclu ve kademeli fren.
            accel = 1.00
            decel = 2.80 if (ego_limit is not None or pedestrian_limit is not None or npc_conflict_limit is not None) else 1.20
            if s['speed'] < desired_speed:
                s['speed'] = min(desired_speed, s['speed'] + accel * dt)
            elif s['speed'] > desired_speed:
                s['speed'] = max(desired_speed, s['speed'] - decel * dt)

            # Onu tamamen durduran bir arac varsa NPC ayni waypointte bekler.
            if s['speed'] < 0.03:
                continue

            wp=s['wp']; s['progress']+=s['speed']*dt
            while s['progress']>=2.0:
                nxt=self._next(wp)
                if nxt is None: break
                wp=nxt; s['wp']=wp; s['progress']-=2.0
            nxt=self._next(wp)
            if nxt is None: continue
            a=wp.transform; b=nxt.transform; t=max(0,min(1,s['progress']/2.0))
            x=a.location.x+(b.location.x-a.location.x)*t
            y=a.location.y+(b.location.y-a.location.y)*t
            z=a.location.z+(b.location.z-a.location.z)*t+0.35
            yaw=math.degrees(math.atan2(b.location.y-a.location.y,b.location.x-a.location.x))
            actor.set_transform(carla.Transform(carla.Location(x=x,y=y,z=z),carla.Rotation(yaw=yaw)))

        if print_npc_debug and npc_debug:
            print(">>> NPC SPEED km/h [anlik/hedef]: " + " | ".join(npc_debug) + " <<<")

    def distance_to(self,ego_vehicle):
        try:
            tf=ego_vehicle.get_transform(); e=tf.location
            f=tf.get_forward_vector(); r=tf.get_right_vector()
        except Exception: return float('inf')
        best=float('inf')
        for a in self.actors:
            try:
                if not a.is_alive: continue
                p=a.get_location(); dx=p.x-e.x; dy=p.y-e.y
                longitudinal=dx*f.x+dy*f.y; lateral=abs(dx*r.x+dy*r.y)

                # Adaptive-follow yalnizca bizimle AYNI yon/seritte ilerleyen
                # araca uygulanir. Karsi seritten gelen arac kamera koridorunda
                # gorunse bile bizi gereksiz yere durdurmaz. Kavsak/capraz
                # guvenligi CampusTrafficScenario'nun collision tahmininde kalir.
                atf = a.get_transform().get_forward_vector()
                same_direction = (f.x * atf.x + f.y * atf.y) > 0.35
                if (same_direction and
                        0 < longitudinal < CAMPUS_NPC_FRONT_MAX_DISTANCE and
                        lateral < CAMPUS_NPC_FRONT_LATERAL_LIMIT):
                    best=min(best,math.hypot(dx,dy))
            except Exception: pass
        return best

    def active_count(self):
        return sum(1 for a in self.actors if a is not None and a.is_alive)

    def destroy(self):
        for a in self.actors:
            try:
                if a.is_alive: a.destroy()
            except Exception: pass
        self.actors.clear(); self.states.clear()


class AmbientWalker:
    """Bir kez karsiya gecen, sonra yol kenarindan uzaklasip duran kinematik yaya."""

    def __init__(self, actor, start_location, target_location, speed):
        self.actor = actor
        self.start = carla.Location(x=start_location.x, y=start_location.y, z=start_location.z)
        self.target = carla.Location(x=target_location.x, y=target_location.y, z=target_location.z)
        self.speed = speed
        self.arrival_radius = 0.35
        self.finished = False
        self.last_update = time.perf_counter()

    def update(self):
        if self.actor is None or not self.actor.is_alive or self.finished:
            return

        loc = self.actor.get_location()
        dx = self.target.x - loc.x
        dy = self.target.y - loc.y
        planar = math.hypot(dx, dy)
        if planar <= self.arrival_radius:
            self.finished = True
            # Hedefe ulasan yaya yolun icinde hareketsiz actor olarak kalmasin.
            # Crossing hedefi zaten karsi yol kenarinin disindadir; normal yaya
            # da yuruyusunu bitirdiginde sahneden temizlenir.
            try:
                if self.actor is not None and self.actor.is_alive:
                    self.actor.destroy()
            except Exception:
                pass
            return

        now = time.perf_counter()
        dt = max(0.01, min(now - self.last_update, 0.12))
        self.last_update = now
        step = min(planar, self.speed * dt)
        nx = loc.x + (dx / planar) * step
        ny = loc.y + (dy / planar) * step
        yaw = math.degrees(math.atan2(dy, dx))
        self.actor.set_transform(
            carla.Transform(
                carla.Location(x=nx, y=ny, z=self.start.z),
                carla.Rotation(yaw=yaw),
            )
        )


class CampusPedestrianScenario:
    """Daha dogal Campus Life yayalari.

    Custom haritada navmesh'e guvenmek yerine WalkerControl kullanilir.
    Yayalar driving waypoint'in hemen yol kenarinda dogar. Bir bolumu yol
    boyunca yurur, bir bolumu ise iki yol kenari arasinda gercek crossing yapar.
    Ego'nun controller/collision koduna dokunulmaz.
    """

    EDGE_MIN = 2.8
    EDGE_MAX = 3.8
    CROSSING_RATIO = 0.35

    def __init__(self, world, ego_route, count=CAMPUS_PEDESTRIAN_COUNT, ego_vehicle=None):
        self.world = world
        self.ego_route = ego_route
        self.ego_vehicle = ego_vehicle
        self.walkers = []
        self.pending_crossings = []
        self._prepare_route_crossings(2)
        # Iki rota crossing'i ego yaklasinca spawn edilir. Diger yayalar
        # kampus icinde hemen yurur; yol kenarinda bos bos beklemez.
        self._spawn(max(0, count - len(self.pending_crossings)))

    def _prepare_route_crossings(self, wanted=2):
        if not self.ego_route or len(self.ego_route) < 80:
            return
        carla_map = self.world.get_map()
        # Yolculugun basina cok yakin degil; kullanicinin gorebilecegi iki
        # farkli bolgede kontrollu crossing olayi.
        for frac in (0.30, 0.62, 0.78):
            if len(self.pending_crossings) >= wanted:
                break
            idx = min(len(self.ego_route)-12, max(18, int((len(self.ego_route)-1)*frac)))
            try:
                wp = carla_map.get_waypoint(
                    self.ego_route[idx], project_to_road=True, lane_type=carla.LaneType.Driving
                )
                if wp is not None:
                    self.pending_crossings.append(wp)
            except Exception:
                pass

    def _spawn_triggered_crossing(self, wp):
        try:
            bps = list(self.world.get_blueprint_library().filter('walker.pedestrian.*'))
            if not bps:
                return False
            tf = wp.transform
            right = tf.get_right_vector()
            side = random.choice((-1.0, 1.0))
            edge = random.uniform(self.EDGE_MIN, self.EDGE_MAX)
            start = carla.Location(
                x=tf.location.x + right.x*edge*side,
                y=tf.location.y + right.y*edge*side,
                z=tf.location.z + 0.45,
            )
            opposite = random.uniform(self.EDGE_MIN, self.EDGE_MAX)
            cross_distance = edge + opposite + 2.5
            dest = carla.Location(
                x=start.x - right.x*cross_distance*side,
                y=start.y - right.y*cross_distance*side,
                z=start.z,
            )
            dx, dy = dest.x-start.x, dest.y-start.y
            bp = random.choice(bps)
            if bp.has_attribute('is_invincible'):
                bp.set_attribute('is_invincible', 'false')
            actor = self.world.try_spawn_actor(
                bp, carla.Transform(start, carla.Rotation(yaw=math.degrees(math.atan2(dy,dx))))
            )
            if actor is None:
                return False
            try:
                actor.set_simulate_physics(False); actor.set_enable_gravity(False)
            except Exception:
                pass
            walker = AmbientWalker(actor, start, dest, random.uniform(1.50,1.80))
            self.walkers.append(walker)
            print('>>> ROTA YAYASI AKTIF - KONTROLLU CROSSING BASLADI <<<')
            return True
        except Exception:
            return False

    def _make_anchor_pool(self):
        carla_map = self.world.get_map()
        anchors = []

        # Kampusun geneline dagilim.
        try:
            anchors.extend(list(carla_map.generate_waypoints(12.0)))
        except Exception:
            pass

        # Ego'nun yolculugu boyunca da hayat gorunsun; bu waypointler sadece
        # spawn bolgesi icindir. Yayanin davranisi ego tarafindan yonetilmez.
        if self.ego_route:
            step = max(10, len(self.ego_route) // 30)
            for i in range(12, max(13, len(self.ego_route) - 12), step):
                try:
                    wp = carla_map.get_waypoint(
                        self.ego_route[i],
                        project_to_road=True,
                        lane_type=carla.LaneType.Driving,
                    )
                    if wp is not None:
                        # Route yakinindaki adaylari havuza birkac kez koyarak
                        # ego'nun en azindan bazi yayalari gorme sansini arttir.
                        anchors.extend([wp, wp, wp, wp])
                except Exception:
                    pass

        random.shuffle(anchors)
        return anchors

    def _spawn(self, requested):
        walker_bps = list(self.world.get_blueprint_library().filter("walker.pedestrian.*"))
        if not walker_bps:
            print("CAMPUS LIFE: walker blueprint bulunamadi.")
            return

        used = []
        for wp in self._make_anchor_pool():
            if len(self.walkers) >= requested:
                break

            try:
                tf = wp.transform
                fwd = tf.get_forward_vector()
                right = tf.get_right_vector()

                # Ayni noktaya insan yigilmamasi.
                if any(distance_2d(tf.location, p) < 10.0 for p in used):
                    continue

                side = random.choice((-1.0, 1.0))
                edge = random.uniform(self.EDGE_MIN, self.EDGE_MAX)

                start = carla.Location(
                    x=tf.location.x + right.x * edge * side,
                    y=tf.location.y + right.y * edge * side,
                    z=tf.location.z + 0.45,
                )

                # Aracin burnunda / hareketli NPC'nin hemen yaninda yaya dogmasin.
                if self.ego_vehicle is not None:
                    try:
                        if distance_2d(start, self.ego_vehicle.get_location()) < 24.0:
                            continue
                    except Exception:
                        pass
                unsafe_npc = False
                try:
                    for v in self.world.get_actors().filter('vehicle.*'):
                        if v.id == getattr(self.ego_vehicle, 'id', -1):
                            continue
                        role = v.attributes.get('role_name', '') if hasattr(v, 'attributes') else ''
                        if role == 'campus_npc' and distance_2d(start, v.get_location()) < 10.0:
                            unsafe_npc = True
                            break
                except Exception:
                    pass
                if unsafe_npc:
                    continue

                crossing = random.random() < self.CROSSING_RATIO

                if crossing:
                    # Bir yol kenarindan DIGER yol kenarina. Eski V1'deki
                    # 8-11 m sabit atlama yerine lane genisligine yakin olcek.
                    try:
                        lane_width = max(3.0, float(wp.lane_width))
                    except Exception:
                        lane_width = 3.5

                    opposite_edge = random.uniform(self.EDGE_MIN, self.EDGE_MAX)
                    # Karsiya gecince 2.5 m daha devam eder; yol kenarinda geri donmez.
                    cross_distance = edge + opposite_edge + 2.5
                    dest = carla.Location(
                        x=start.x - right.x * cross_distance * side,
                        y=start.y - right.y * cross_distance * side,
                        z=start.z,
                    )
                else:
                    # Yol kenarinda yol dogrultusunda yurume.
                    travel = random.uniform(24.0, 45.0)
                    direction_sign = random.choice((-1.0, 1.0))
                    dest = carla.Location(
                        x=start.x + fwd.x * travel * direction_sign,
                        y=start.y + fwd.y * travel * direction_sign,
                        z=start.z,
                    )

                dx = dest.x - start.x
                dy = dest.y - start.y
                yaw = math.degrees(math.atan2(dy, dx))

                bp = random.choice(walker_bps)
                if bp.has_attribute("is_invincible"):
                    bp.set_attribute("is_invincible", "false")

                actor = self.world.try_spawn_actor(
                    bp,
                    carla.Transform(start, carla.Rotation(yaw=yaw)),
                )
                if actor is None:
                    continue
                try:
                    actor.set_simulate_physics(False)
                    actor.set_enable_gravity(False)
                except Exception:
                    pass

                speed = random.uniform(1.50, 1.80)
                walker = AmbientWalker(actor, start, dest, speed)
                walker.update()
                self.walkers.append(walker)
                used.append(tf.location)

            except Exception:
                continue

        print(
            f">>> CAMPUS LIFE V7: {len(self.walkers)}/{requested} KINEMATIK HAREKETLI YAYA AKTIF "
            f"(yaklasik %{int(self.CROSSING_RATIO*100)} crossing) <<<"
        )

    def update(self, vehicle=None):
        ego = vehicle if vehicle is not None else self.ego_vehicle

        # Garantili crossing yayalari yol kenarinda dakikalarca beklemez.
        # Ego 24-38 m bandina geldiginde actor yeni spawn olur ve hemen
        # karsiya gecmeye baslar. Boylece testte gercek karsilasma olusur.
        if ego is not None and getattr(ego, 'is_alive', False):
            try:
                eloc = ego.get_location()
                remaining = []
                for wp in self.pending_crossings:
                    d = distance_2d(eloc, wp.transform.location)
                    if 24.0 <= d <= 38.0:
                        if not self._spawn_triggered_crossing(wp):
                            remaining.append(wp)
                    elif d > 24.0:
                        remaining.append(wp)
                    # d < 24 ise nokta kacirildi; gec spawn edip aracin burnuna
                    # yaya cikarmiyoruz.
                self.pending_crossings = remaining
            except Exception:
                pass

        for walker in self.walkers:
            try:
                walker.update()
            except Exception:
                pass

    def active_count(self):
        return sum(
            1 for w in self.walkers
            if w.actor is not None and getattr(w.actor, "is_alive", False)
        )

    def destroy(self):
        for walker in self.walkers:
            try:
                if walker.actor is not None and walker.actor.is_alive:
                    walker.actor.destroy()
            except Exception:
                pass
        self.walkers.clear()


# ============================================================
# SABIT HARITA COLLISION BOLGESI - ROTA BAZLI KORUMA
# ============================================================

BAD_COLLISION_CENTER = carla.Location(
    x=347.0,
    y=-911.75,
    z=0.0,
)

# Rota bu merkezin bu kadar yakinindan gecerse physics kullanma.
BAD_COLLISION_ROUTE_RADIUS = 8.0


def route_hits_bad_collision_zone(route_locations):
    if not route_locations:
        return False, None, float("inf")

    best_index = None
    best_distance = float("inf")

    for index, location in enumerate(route_locations):
        d = distance_2d(location, BAD_COLLISION_CENTER)
        if d < best_distance:
            best_distance = d
            best_index = index

    return (
        best_distance <= BAD_COLLISION_ROUTE_RADIUS,
        best_index,
        best_distance,
    )


def drive_full_kinematic(
    world,
    vehicle,
    route_locations,
    vision_system=None,
    pedestrian_scenario=None,
    lead_scenario=None,
    base_speed_kmh=ANAGIRIS_KINEMATIC_SPEED_KMH,
):
    """
    Bu rota fiziksel collision riski tasiyorsa physics HIC geri acilmaz.
    Arac mevcut A* rotasini kinematik olarak hedefe kadar takip eder.
    V5.4 controller dosyasina dokunulmaz.
    """

    print(
        "\n======================================"
    )
    print(
        "COLLISION-SAFE FULL KINEMATIC MODE AKTIF"
    )
    print(
        "======================================"
    )
    print(
        "Physics OFF -> rota boyunca kinematik -> hedefte dur."
    )
    print(
        "V5.4 vehicle_controller.py DEGISTIRILMEDI."
    )

    if len(route_locations) < 2:
        raise RuntimeError(
            "Kinematik surus icin rota yetersiz."
        )

    vehicle.apply_control(
        carla.VehicleControl(
            throttle=0.0,
            steer=0.0,
            brake=1.0,
        )
    )

    time.sleep(0.15)

    transform = vehicle.get_transform()
    yaw = transform.rotation.yaw

    cumulative = _route_cumulative(
        route_locations
    )

    total_distance = cumulative[-1]

    # Spawn edilen aracin gercek Z'sini rota Z'sine gore koru.
    z_delta = (
        transform.location.z
        - route_locations[0].z
    )

    z_delta = max(
        ANAGIRIS_KINEMATIC_Z_OFFSET_MIN,
        min(
            ANAGIRIS_KINEMATIC_Z_OFFSET_MAX,
            z_delta,
        ),
    )

    # Ana Giris icin eski guvenli 7 km/h korunur. Diger collision-safe
    # segmentlerde physics yine OFF kalir fakat normal kampus hizi kullanilabilir.
    speed_mps = (
        base_speed_kmh
        / 3.6
    )

    step_distance = (
        speed_mps
        * ANAGIRIS_KINEMATIC_STEP
    )

    travelled = 0.0
    last_print = 0.0
    pedestrian_safety = PedestrianSafety()
    vehicle_following = VehicleFollowingSafety()

    vehicle.set_simulate_physics(False)
    vehicle.set_enable_gravity(False)

    # Kinematik ilerleme artik sabit 0.05 s varsayimina bagli degil.
    # CARLA/Unreal agirladiginda dongu 50 ms'den uzun surerse eski kod
    # araci gercek zamana gore gereksiz yavaslatiyordu. Gercek gecen zamani
    # kullanmak hedef hizi korur; 20 ms nominal adim da set_transform
    # hareketini daha akici yapar. Collision mantigi DEGISTIRILMEDI.
    last_motion_time = time.perf_counter()

    try:

        while travelled < total_distance:

            now_motion = time.perf_counter()
            motion_dt = max(0.005, min(now_motion - last_motion_time, 0.08))
            last_motion_time = now_motion

            # MAIN 8: kinematik modda fren komutu tek basina ise yaramaz;
            # bu nedenle yaya varken rota ilerlemesini tamamen donduruyoruz.
            if pedestrian_scenario is not None:
                pedestrian_scenario.update(vehicle)
            if lead_scenario is not None:
                lead_scenario.update()

            pedestrian_level = pedestrian_safety.update(
                vision_system, vehicle, pedestrian_scenario
            )

            lead_distance = (
                lead_scenario.distance_to(vehicle)
                if lead_scenario is not None else float("inf")
            )
            vehicle_following.update(vision_system, lead_distance)
            follow_limit, follow_state = vehicle_following.speed_limit(
                base_speed_kmh,
                lead_distance,
            )
            pedestrian_limit = pedestrian_safety.speed_limit(base_speed_kmh)
            combined_limit = min(follow_limit, pedestrian_limit)
            adaptive_step_distance = (combined_limit / 3.6) * motion_dt
            if adaptive_step_distance <= 0.0:
                update_spectator_camera(world, vehicle)
                time.sleep(ANAGIRIS_KINEMATIC_STEP)
                continue

            travelled = min(
                total_distance,
                travelled + adaptive_step_distance,
            )

            location, route_index = (
                _sample_route(
                    route_locations,
                    cumulative,
                    travelled,
                )
            )

            ahead_distance = min(
                total_distance,
                travelled + 2.0,
            )

            ahead, _ = _sample_route(
                route_locations,
                cumulative,
                ahead_distance,
            )

            desired_yaw = math.degrees(
                math.atan2(
                    ahead.y - location.y,
                    ahead.x - location.x,
                )
            )

            yaw = _lerp_angle(
                yaw,
                desired_yaw,
                ANAGIRIS_KINEMATIC_YAW_ALPHA,
            )

            # Lateral offset YOK:
            # arac dogrudan mevcut A* rota merkezini takip eder.
            safe_location = carla.Location(
                x=location.x,
                y=location.y,
                z=location.z + z_delta,
            )

            vehicle.set_transform(
                carla.Transform(
                    safe_location,
                    carla.Rotation(
                        pitch=0.0,
                        yaw=yaw,
                        roll=0.0,
                    ),
                )
            )

            # Kamera her adimda araci takip eder.
            update_spectator_camera(
                world,
                vehicle,
            )

            # YOLO sadece goruntu/detection. Arac kontrolune mudahale etmez.

            now = time.time()

            if (
                now - last_print
                >= PRINT_INTERVAL
            ):

                remaining = max(
                    0.0,
                    total_distance - travelled,
                )

                print(
                    "KINEMATIK | Hiz:",
                    f"{combined_limit:.1f}",
                    "km/h",
                    "| Follow:",
                    follow_state,
                    "| Lead:",
                    f"{lead_distance:.1f} m",
                    "| Kalan:",
                    f"{remaining:.1f}",
                    "m",
                    "| Route:",
                    f"{route_index}/{len(route_locations) - 1}",
                )

                last_print = now

            time.sleep(
                ANAGIRIS_KINEMATIC_STEP
            )

        # Tam hedef transformu.
        goal = route_locations[-1]

        if len(route_locations) >= 2:
            before_goal = route_locations[-2]
            final_yaw = heading_between(
                before_goal,
                goal,
            )
        else:
            final_yaw = yaw

        vehicle.set_transform(
            carla.Transform(
                carla.Location(
                    x=goal.x,
                    y=goal.y,
                    z=goal.z + z_delta,
                ),
                carla.Rotation(
                    pitch=0.0,
                    yaw=final_yaw,
                    roll=0.0,
                ),
            )
        )

        update_spectator_camera(
            world,
            vehicle,
        )

        print(
            "\n======================================"
        )
        print(
            "ANA GIRIS KINEMATIK: HEDEFE ULASILDI"
        )
        print(
            "======================================"
        )

    finally:
        # BILEREK physics geri acilmiyor.
        # Ana Giris rotasindaki problemli fizik bolgeleri tekrar araci
        # firlatmasin diye hedefte de kinematik kalir.
        try:
            vehicle.apply_control(
                carla.VehicleControl(
                    throttle=0.0,
                    steer=0.0,
                    brake=1.0,
                )
            )
        except Exception:
            pass


# ============================================================
# NORMAL PHYSICS SURUSU - VIRAJ ONGORU HIZ KATMANI
# ============================================================

def predictive_curve_speed_limit(route_locations, route_index, base_speed_kmh=14.0):
    """
    Frozen V5.4 controller'a dokunmadan, yaklasan viraji ONCEDEN gorur.

    A* rota noktalarinin onumuzdeki yaklasik 26 metresindeki toplam yon
    degisimini olcer. Boylece controller VIRAJ/KESKIN VIRAJ durumuna tam
    girdigi anda degil, viraja gelmeden once external_speed_limit dusurulur.

    Collision / kinematic workaround ile ilgisi yoktur; yalnizca normal
    physics drive_route() icin ek hiz ust siniridir.
    """
    if not route_locations or len(route_locations) < 3:
        return float(base_speed_kmh), "DUZ-ONGORU"

    start = max(0, min(int(route_index), len(route_locations) - 2))
    max_lookahead_m = 26.0
    travelled = 0.0
    total_turn_deg = 0.0
    max_local_turn_deg = 0.0

    prev_heading = None
    prev = route_locations[start]

    for i in range(start + 1, len(route_locations)):
        cur = route_locations[i]
        dx = cur.x - prev.x
        dy = cur.y - prev.y
        seg = math.hypot(dx, dy)
        if seg < 0.05:
            prev = cur
            continue

        travelled += seg
        heading = math.degrees(math.atan2(dy, dx))

        if prev_heading is not None:
            delta = (heading - prev_heading + 180.0) % 360.0 - 180.0
            ad = abs(delta)
            # Haritadaki cok kisa/noisy waypoint sapmalarini bastir.
            if ad < 65.0:
                total_turn_deg += ad
                max_local_turn_deg = max(max_local_turn_deg, ad)

        prev_heading = heading
        prev = cur

        if travelled >= max_lookahead_m:
            break

    # Keskin viraj: daha viraja girmeden 5.5 km/h seviyesine hazirlan.
    if total_turn_deg >= 42.0 or max_local_turn_deg >= 24.0:
        return min(float(base_speed_kmh), 5.5), "KESKIN-VIRAJ-ONGORU"

    # Orta viraj.
    if total_turn_deg >= 24.0 or max_local_turn_deg >= 14.0:
        return min(float(base_speed_kmh), 8.0), "VIRAJ-ONGORU"

    # Hafif viraj: yumusak bir on-yavaslama.
    if total_turn_deg >= 11.0 or max_local_turn_deg >= 7.0:
        return min(float(base_speed_kmh), 11.0), "HAFIF-VIRAJ-ONGORU"

    return float(base_speed_kmh), "DUZ-ONGORU"


# ============================================================
# SÜRÜŞ
# ============================================================

def drive_route(
    world,
    vehicle,
    route_locations,
    vision_system=None,
    pedestrian_scenario=None,
    lead_scenario=None,
):

    print(
        "\n=============================="
    )

    print(
        "       OTONOM SURUS"
    )

    print(
        "=============================="
    )

    controller = (
        VehicleController()
    )

    controller.reset()

    pedestrian_safety = PedestrianSafety()
    vehicle_following = VehicleFollowingSafety()
    last_print_time = 0.0
    last_route_index = 0

    while True:

        # ----------------------------------------------------
        # KAMERA
        # ----------------------------------------------------

        update_spectator_camera(
            world,
            vehicle,
        )

        # YOLO sadece goruntu/detection. Arac kontrolune mudahale etmez.

        # ----------------------------------------------------
        # CONTROLLER
        # ----------------------------------------------------

        if lead_scenario is not None:
            lead_scenario.update()

        lead_distance = (
            lead_scenario.distance_to(vehicle)
            if lead_scenario is not None else float("inf")
        )
        vehicle_following.update(vision_system, lead_distance)

        # 14 km/h sadece takip katmaninin ust siniridir. Frozen controller
        # viraj ve durak hizini kendi icinde daha da dusurmeye devam eder.
        follow_limit, follow_state = vehicle_following.speed_limit(
            14.0,
            lead_distance,
        )

        # Yaya katmani artik kademeli: algila -> yavasla -> dur.
        if pedestrian_scenario is not None:
            pedestrian_scenario.update(vehicle)
        pedestrian_level = pedestrian_safety.update(
                vision_system, vehicle, pedestrian_scenario
            )
        pedestrian_limit = pedestrian_safety.speed_limit(14.0)

        # V4 CURVE-SAFE: viraji controller durum degistirmeden ONCE gor.
        # Bu katman sadece normal physics surusunde calisir. Collision-safe
        # kinematik modlara ve BAD_COLLISION workaround'una dokunmaz.
        curve_limit, curve_preview_state = predictive_curve_speed_limit(
            route_locations,
            last_route_index,
            14.0,
        )

        combined_limit = min(follow_limit, pedestrian_limit, curve_limit)

        # Frozen V5.4 API'sindeki external_speed_limit kullanilir;
        # controller dosyasinin kendisi degistirilmez.
        result = controller.control(
            vehicle,
            route_locations,
            external_speed_limit=combined_limit,
        )
        last_route_index = result.get("route_index", last_route_index)

        # ----------------------------------------------------
        # DEBUG:
        # Controller'ın o anda ne hedeflediğini çiz.
        # ----------------------------------------------------

        draw_dynamic_controller_debug(
            world,
            vehicle,
            route_locations,
            result,
        )

        # ----------------------------------------------------
        # KONTROLÜ UYGULA
        # ----------------------------------------------------

        # STOP seviyesinde tam fren. CAUTION/DANGER hizlari controller'a
        # combined_limit olarak verildi; direksiyon A* / Pure Pursuit'te kalir.
        if pedestrian_level == "STOP":
            vehicle.apply_control(
                carla.VehicleControl(
                    throttle=0.0,
                    steer=result["control"].steer,
                    brake=1.0,
                    hand_brake=False,
                    reverse=False,
                )
            )
        else:
            vehicle.apply_control(result["control"])

        # ----------------------------------------------------
        # TERMINAL
        # ----------------------------------------------------

        now = time.time()

        if (
            now
            - last_print_time
            >= PRINT_INTERVAL
        ):

            vehicle_location = (
                vehicle.get_location()
            )

            target_location = (
                route_locations[
                    result["target_index"]
                ]
            )

            target_distance = (
                distance_2d(
                    vehicle_location,
                    target_location,
                )
            )

            print(
                "Hiz:",
                f'{result["current_speed"]:.1f}',
                "km/h",
                "| Hedef:",
                f'{result["target_speed"]:.1f}',
                "km/h",
                "| Durum:",
                result["road_state"],
                "| Preview:",
                curve_preview_state,
                "| CurveLimit:",
                f"{curve_limit:.1f}",
                "| Follow:",
                follow_state,
                "| Lead:",
                f"{lead_distance:.1f} m",
                "| Kalan:",
                f'{result["remaining_distance"]:.1f}',
                "m",
                "| Route:",
                f'{result["route_index"]}/{len(route_locations) - 1}',
                "| Target:",
                result["target_index"],
                "| TargetDist:",
                f"{target_distance:.1f}",
                "m",
            )

            last_print_time = (
                now
            )

        # ----------------------------------------------------
        # HEDEFE GELDİ
        # ----------------------------------------------------

        if result["arrived"]:

            vehicle.apply_control(
                carla.VehicleControl(
                    throttle=0.0,
                    steer=0.0,
                    brake=1.0,
                    hand_brake=False,
                    reverse=False,
                )
            )

            update_spectator_camera(
                world,
                vehicle,
            )

            print(
                "\n=============================="
            )

            print(
                "      HEDEFE ULASILDI"
            )

            print(
                "=============================="
            )

            break

        time.sleep(
            CONTROL_PERIOD
        )



# ============================================================
# COLLISION SENSOR - GECICI TEShis
# ============================================================

def attach_collision_sensor(
    world,
    vehicle,
):
    """
    Araca CARLA collision sensor baglar.
    Collision event gelirse terminale actor, impulse ve konum yazar.
    """

    blueprint_library = (
        world.get_blueprint_library()
    )

    collision_bp = (
        blueprint_library.find(
            "sensor.other.collision"
        )
    )

    collision_sensor = world.spawn_actor(
        collision_bp,
        carla.Transform(),
        attach_to=vehicle,
    )

    def on_collision(event):

        other_actor = event.other_actor
        impulse = event.normal_impulse

        impulse_magnitude = math.sqrt(
            impulse.x ** 2
            + impulse.y ** 2
            + impulse.z ** 2
        )

        vehicle_location = (
            vehicle.get_location()
        )

        print("\n" + "!" * 70)
        print("COLLISION SENSOR EVENT")
        print("Frame:", event.frame)

        print(
            "Other actor:",
            getattr(
                other_actor,
                "type_id",
                "unknown",
            ),
        )

        print(
            "Other actor id:",
            getattr(
                other_actor,
                "id",
                "unknown",
            ),
        )

        print(
            "Impulse:",
            f"{impulse_magnitude:.3f}",
        )

        print(
            "Vehicle location:",
            f"X={vehicle_location.x:.3f}",
            f"Y={vehicle_location.y:.3f}",
            f"Z={vehicle_location.z:.3f}",
        )

        print("!" * 70 + "\n")

    collision_sensor.listen(
        on_collision
    )

    print(
        "\nCollision sensor baglandi."
    )

    print(
        "Bir carpisma olursa terminalde "
        "'COLLISION SENSOR EVENT' goreceksin."
    )

    return collision_sensor


# ============================================================
# MULTI-STOP DURAK SECIMI
# ============================================================

INTERMEDIATE_STOP_WAIT_SECONDS = 2.0


def choose_multi_stop_journey():
    """Kullanicidan yolculuktaki duraklari SIRASIYLA alir.

    2 durak secilirse eski Baslangic -> Hedef davranisinin aynisidir.
    3+ durakta her ardisik durak cifti mevcut A* / special-route sistemiyle
    ayri ayri planlanir: D1->D2, D2->D3, ...
    """
    stops = list(BUS_STOPS.values())
    if len(stops) < 2:
        raise RuntimeError("En az 2 durak tanimli olmali.")

    print("\n==============================")
    print("     IKCU MULTI-STOP ROTA")
    print("==============================")
    for index, stop in enumerate(stops, start=1):
        print(f"{index}) {stop.name}")

    while True:
        try:
            count = int(input(f"\nKac duraklik yolculuk? (2-{len(stops)}): "))
            if 2 <= count <= len(stops):
                break
        except ValueError:
            pass
        print("Gecersiz sayi. Tekrar dene.")

    selected = []
    for order in range(1, count + 1):
        while True:
            try:
                choice = int(input(f"{order}. durak numarasi: "))
                if not (1 <= choice <= len(stops)):
                    raise ValueError
                stop = stops[choice - 1]
                if any(s.name == stop.name for s in selected):
                    print("Bu durak zaten secildi. Farkli bir durak sec.")
                    continue
                selected.append(stop)
                print(f"  -> {stop.name}")
                break
            except ValueError:
                print("Gecersiz secim. Tekrar dene.")

    print("\n==============================")
    print("SECILEN DURAK ZINCIRI")
    print("==============================")
    print(" -> ".join(stop.name for stop in selected))
    print("==============================")
    return selected


def combine_route_segments(route_segments):
    """Campus-life dagilimi/debug icin segmentleri tek rota listesinde birlestirir."""
    combined = []
    for segment in route_segments:
        if not segment:
            continue
        if combined and distance_2d(combined[-1], segment[0]) < 0.25:
            combined.extend(segment[1:])
        else:
            combined.extend(segment)
    return combined


def wait_at_intermediate_stop(
    world,
    vehicle,
    seconds,
    pedestrian_scenario=None,
    lead_scenario=None,
    kinematic=False,
):
    """Ara durakta ego sabit kalirken kampus hayati calismaya devam eder."""
    print(f">>> DURAKTA {seconds:.0f} SANIYE BEKLENIYOR <<<")
    end_time = time.perf_counter() + seconds

    while time.perf_counter() < end_time:
        if not kinematic:
            try:
                vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        steer=0.0,
                        brake=1.0,
                        hand_brake=False,
                        reverse=False,
                    )
                )
            except Exception:
                pass

        if pedestrian_scenario is not None:
            pedestrian_scenario.update(vehicle)
        if lead_scenario is not None:
            lead_scenario.update()

        update_spectator_camera(world, vehicle)
        time.sleep(CONTROL_PERIOD)


# ============================================================
# MAIN
# ============================================================

def main():

    print("\n>>> MAIN V8 MULTI-STOP - EXISTING SAFETY SYSTEM PRESERVED <<<")

    vehicle = None
    collision_sensor = None
    vision_system = None
    pedestrian_scenario = None
    lead_scenario = None

    try:
        print("\nCARLA'ya baglaniliyor...")
        client = carla.Client(CARLA_HOST, CARLA_PORT)
        client.set_timeout(CARLA_TIMEOUT)
        world = client.get_world()
        carla_map = world.get_map()

        print("CARLA baglantisi basarili.")
        print("Harita:", carla_map.name)

        # ----------------------------------------------------
        # MULTI-STOP SECIM
        # ----------------------------------------------------
        selected_stops = choose_multi_stop_journey()
        start_stop = selected_stops[0]
        goal_stop = selected_stops[-1]

        print("\nSTART:", start_stop.name)
        print("FINAL GOAL:", goal_stop.name)
        print("Toplam durak:", len(selected_stops))
        print("Toplam segment:", len(selected_stops) - 1)

        # ----------------------------------------------------
        # SPAWN - ESKI SISTEM AYNI
        # ----------------------------------------------------
        print("\nArac spawn ediliyor...")
        spawn_result = spawn_ego_near_stop(world, start_stop.location)
        if spawn_result is None:
            raise RuntimeError("Arac spawn edilemedi.")

        vehicle = spawn_result["vehicle"]
        update_spectator_camera(world, vehicle)

        collision_sensor = attach_collision_sensor(world, vehicle)

        vision_system = VisionSystem(world, vehicle)
        vision_system.start()
        print("\nYOLO + MAIN10 MULTI OBJECT SAFETY aktif.")

        actual_start_location = vehicle.get_location()
        print("\nSpawn basarili.")
        print("Spawn yontemi:", spawn_result.get("method", "unknown"))
        print("Duraktan uzaklik:", f'{spawn_result.get("distance", 0.0):.2f}', "m")

        planner = RoutePlanner(carla_map)

        # ----------------------------------------------------
        # HER ARDISIK DURAK CIFTI ICIN AYNI A* SISTEMI
        # ----------------------------------------------------
        route_segments = []

        for i in range(len(selected_stops) - 1):
            segment_start = selected_stops[i]
            segment_goal = selected_stops[i + 1]

            # Ilk segment gercek spawn konumundan baslar. Sonraki segmentlerde
            # ilgili ara duragin koordinati kullanilir; suruste arac zaten o
            # duraga ulasmis olacaktir.
            segment_start_location = (
                actual_start_location if i == 0 else segment_start.location
            )

            print("\n======================================")
            print(f"ROTA {i + 1}/{len(selected_stops) - 1}: {segment_start.name} -> {segment_goal.name}")
            print("======================================")

            segment_route = plan_special_route(
                planner,
                BUS_STOPS,
                segment_start.name,
                segment_goal.name,
                start_location=segment_start_location,
            )

            segment_route = apply_collision_bypass(planner, segment_route)

            if segment_route is None or len(segment_route) < 2:
                raise RuntimeError(
                    f"Gecerli rota bulunamadi: {segment_start.name} -> {segment_goal.name}"
                )

            route_segments.append(segment_route)
            print("Segment node:", len(segment_route))

        full_route = combine_route_segments(route_segments)
        if len(full_route) < 2:
            raise RuntimeError("Birlestirilmis multi-stop rota gecersiz.")

        # ----------------------------------------------------
        # CAMPUS LIFE - TEK KEZ KURULUR, TUM YOLCULUKTA KALIR
        # ----------------------------------------------------
        lead_scenario = CampusTrafficScenario(
            client,
            world,
            vehicle,
            CAMPUS_NPC_VEHICLE_COUNT,
            full_route,
        )

        pedestrian_scenario = CampusPedestrianScenario(
            world,
            full_route,
            CAMPUS_PEDESTRIAN_COUNT,
            ego_vehicle=vehicle,
        )
        lead_scenario.set_pedestrian_scenario(pedestrian_scenario)

        print("\n======================================")
        print("FINAL CAMPUS LIFE HAZIR")
        print("Bagimsiz NPC arac sayisi:", lead_scenario.active_count())
        print("Hareketli yaya sayisi:", pedestrian_scenario.active_count())
        print("Oncelik: YAYA > YAKIN ARAC > ADAPTIVE FOLLOW > NORMAL SURUS")
        print("======================================")

        suspicious_segments = validate_route_direction(carla_map, full_route)
        draw_static_route(world, full_route)
        draw_collision_bypass_debug(world, full_route, life_time=30.0)

        initial_heading_error = check_initial_heading(vehicle, route_segments[0])
        if initial_heading_error > INITIAL_HEADING_ABORT_DEG:
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            raise RuntimeError("Arac rota yonunun tersine bakiyor.")

        print("\n==============================")
        print("      MULTI-STOP TEST OZETI")
        print("==============================")
        print("Duraklar:", " -> ".join(stop.name for stop in selected_stops))
        print("Toplam segment:", len(route_segments))
        print("Toplam route node:", len(full_route))
        print("Supheli segment:", len(suspicious_segments))
        print("Heading farki:", f"{initial_heading_error:.1f}")
        print("Ara durak bekleme:", f"{INTERMEDIATE_STOP_WAIT_SECONDS:.0f} sn")
        print("==============================")

        print("\n3 saniye sonra surus baslayacak...")
        for _ in range(30):
            update_spectator_camera(world, vehicle)
            time.sleep(0.1)

        # ----------------------------------------------------
        # SEGMENT SEGMENT SURUS
        # ----------------------------------------------------
        # Kinematik moda bir kez girildiyse physics bilerek geri acilmadigi
        # icin sonraki segmentler de kinematik devam eder.
        kinematic_latched = False

        for i, route_locations in enumerate(route_segments):
            segment_start = selected_stops[i]
            segment_goal = selected_stops[i + 1]

            print("\n" + "=" * 54)
            print(f"SEGMENT {i + 1}/{len(route_segments)}: {segment_start.name} -> {segment_goal.name}")
            print("=" * 54)

            normalized_start_name = (
                str(segment_start.name)
                .strip()
                .lower()
                .replace("ı", "i")
                .replace("İ", "i")
            )

            collision_route, collision_index, collision_distance = (
                route_hits_bad_collision_zone(route_locations)
            )

            print(
                "Collision-zone kontrolu:",
                "EVET" if collision_route else "HAYIR",
                "| En yakin route index:", collision_index,
                "| Mesafe:", f"{collision_distance:.2f} m",
            )

            segment_force_kinematic = (
                kinematic_latched
                or normalized_start_name in ("anagiris", "ana giris")
                or collision_route
            )

            if segment_force_kinematic:
                # Collision cozumunu BOZMUYORUZ: physics kapali kalir ve set_transform
                # tabanli collision-safe surus kullanilir. Ancak 7 km/h sadece eski
                # Ana Giris ozel cozumune aittir. Diger kinematik segmentler 14 km/h
                # taban hizla gider; yaya/lead limitleri yine bunu dusurebilir.
                kinematic_latched = True
                is_anagiris_segment = normalized_start_name in ("anagiris", "ana giris")
                kinematic_base_speed = (
                    ANAGIRIS_KINEMATIC_SPEED_KMH if is_anagiris_segment else 14.0
                )
                drive_full_kinematic(
                    world,
                    vehicle,
                    route_locations,
                    vision_system=vision_system,
                    pedestrian_scenario=pedestrian_scenario,
                    lead_scenario=lead_scenario,
                    base_speed_kmh=kinematic_base_speed,
                )
            else:
                drive_route(
                    world,
                    vehicle,
                    route_locations,
                    vision_system=vision_system,
                    pedestrian_scenario=pedestrian_scenario,
                    lead_scenario=lead_scenario,
                )

            # Son durak degilse TAM 2 saniye bekle ve sonraki A* segmente gec.
            if i < len(route_segments) - 1:
                print("\n======================================")
                print("ARA DURAGA ULASILDI:", segment_goal.name)
                print("======================================")
                wait_at_intermediate_stop(
                    world,
                    vehicle,
                    INTERMEDIATE_STOP_WAIT_SECONDS,
                    pedestrian_scenario=pedestrian_scenario,
                    lead_scenario=lead_scenario,
                    kinematic=kinematic_latched,
                )
                print(">>> SONRAKI DURAGA DEVAM <<<")

        print("\n======================================")
        print("MULTI-STOP YOLCULUK TAMAMLANDI")
        print("SON DURAK:", goal_stop.name)
        print("======================================")

        print("\nArac son durakta 5 saniye bekliyor...")
        for _ in range(50):
            update_spectator_camera(world, vehicle)
            if pedestrian_scenario is not None:
                pedestrian_scenario.update(vehicle)
            if lead_scenario is not None:
                lead_scenario.update()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nTest kullanici tarafindan durduruldu.")

    except Exception as error:
        print("\n==============================")
        print("HATA")
        print("==============================")
        print(error)

    finally:
        if lead_scenario is not None:
            try:
                lead_scenario.destroy()
                print("Campus Life NPC araclari temizlendi.")
            except Exception as error:
                print("Lead vehicle cleanup uyarisi:", error)

        if pedestrian_scenario is not None:
            try:
                pedestrian_scenario.destroy()
            except Exception as error:
                print("Yaya cleanup uyarisi:", error)

        if vision_system is not None:
            try:
                vision_system.destroy()
                print("RGB kamera + YOLO temizlendi.")
            except Exception as error:
                print("Vision cleanup uyarisi:", error)

        if collision_sensor is not None:
            try:
                collision_sensor.stop()
            except Exception:
                pass
            try:
                collision_sensor.destroy()
                print("Collision sensor temizlendi.")
            except Exception:
                pass

        if vehicle is not None:
            try:
                vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        steer=0.0,
                        brake=1.0,
                    )
                )
            except Exception:
                pass

            time.sleep(0.5)
            try:
                destroy_vehicle(vehicle)
                print("Test araci temizlendi.")
            except Exception:
                try:
                    vehicle.destroy()
                except Exception:
                    pass

        print("\nProgram sonlandi.")


# ============================================================
# CALISTIR
# ============================================================

if __name__ == "__main__":
    main()

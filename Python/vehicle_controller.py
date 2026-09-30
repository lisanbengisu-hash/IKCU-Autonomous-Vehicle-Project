import math
import time

import carla

from config import (
    LOOKAHEAD_MIN,
    LOOKAHEAD_MAX,
    WHEEL_BASE,
    MAX_STEER,
    MAX_THROTTLE,
    MAX_BRAKE,
    STEERING_SMOOTHING_ALPHA,
    MAX_ROUTE_SPEED_KMH,
    MIN_CURVE_SPEED_KMH,
    GOAL_APPROACH_SPEED_KMH,
    CURVE_LOOKAHEAD_METERS,
    BUS_STOP_SLOWDOWN_DISTANCE,
    BUS_STOP_TOLERANCE_METERS,
)

SPECIAL_OFFSET_ENABLED = False
SPECIAL_OFFSET_ROUTE_MIN_LENGTH = 580
SPECIAL_OFFSET_START_INDEX = 400
SPECIAL_OFFSET_END_INDEX = 430
SPECIAL_OFFSET_METERS = 1.00


# ============================================================
# PHYSICS BRIDGE - STATIC MAP COLLISION WORKAROUND
# ============================================================

PHYSICS_BRIDGE_ENABLED = False

PHYSICS_BRIDGE_CENTER = carla.Location(
    x=347.0,
    y=-911.75,
    z=0.0,
)

# Collision merkezinden rota boyunca once/sonra
PHYSICS_BRIDGE_ENTRY_METERS = 16.0
PHYSICS_BRIDGE_EXIT_METERS = 55.0

# Physics kapaliyken gorunen gecis hizi
PHYSICS_BRIDGE_SPEED_MPS = 1.8  # 7.2 km/h

# Physics geri acilirken zemine gomulmeyi onlemek icin
PHYSICS_BRIDGE_Z_LIFT = 0.90


# ============================================================
# V5 AUTOMATIC STUCK / COLLISION RECOVERY
# ============================================================

AUTO_RECOVERY_ENABLED = False

# Arac hareket etmesi gerekirken bu hizdan dusuk kalirsa takilma adayi.
AUTO_RECOVERY_STUCK_SPEED_KMH = 0.8

# Hedef hiz bunun altindaysa recovery yapma.
AUTO_RECOVERY_MIN_TARGET_SPEED_KMH = 4.0

# Bu kadar sure gercekten yerinde kalirsa recovery tetiklenir.
AUTO_RECOVERY_STUCK_SECONDS = 1.4

# Aracin "hareket etti" sayilmasi icin minimum XY ilerleme.
AUTO_RECOVERY_MIN_PROGRESS_METERS = 0.40

# Takildigi yerden rota boyunca kac metre ileri atlanacak.
AUTO_RECOVERY_JUMP_AHEAD_METERS = 24.0

# Ilk secilen nokta kotuyse daha ileri alternatifler.
AUTO_RECOVERY_EXTRA_JUMPS = [32.0, 40.0]

# Gercek yuzeyin ustune spawn yuksekligi.
AUTO_RECOVERY_SAFE_ABOVE_GROUND = 1.55

# Iki recovery arasinda minimum sure.
AUTO_RECOVERY_COOLDOWN_SECONDS = 2.5

# Sonsuz recovery dongusunu engeller ama bir rotayi bitirmek icin yeterli.
AUTO_RECOVERY_MAX_COUNT = 12

# ============================================================
# LOCAL EARLY TURN - TARGET 455 CURVE
# ============================================================
EARLY_TURN_ENABLED = True
EARLY_TURN_START_INDEX = 438
EARLY_TURN_END_INDEX = 464
EARLY_TURN_EXTRA_LOOKAHEAD = 4.2
EARLY_TURN_MAX_SPEED_KMH = 4.2


# Problemli virajda direksiyonu biraz daha erken/agresif kir.
EARLY_TURN_STEER_GAIN = 1.18
EARLY_TURN_MIN_LOOKAHEAD = 7.5


# ============================================================
# V5.4 KINEMATIC FINAL CORRIDOR
# ============================================================
# Haritanin bozuk collision koridorundan once devreye girer.
# Bu noktadan hedefe kadar physics KAPALI kalir. Arac A*
# rotasini her frame kucuk adimlarla takip eder; collision onu
# durduramaz veya takla attirmaz.

KINEMATIC_FINAL_ENABLED = True
KINEMATIC_FINAL_START_INDEX = 398

KINEMATIC_STRAIGHT_SPEED_KMH = 10.0
KINEMATIC_LIGHT_CURVE_SPEED_KMH = 8.0
KINEMATIC_CURVE_SPEED_KMH = 6.5
KINEMATIC_SHARP_CURVE_SPEED_KMH = 5.0

KINEMATIC_Z_OFFSET_MIN = -0.30
KINEMATIC_Z_OFFSET_MAX = 0.20
KINEMATIC_YAW_ALPHA = 0.22


# ============================================================
# V5.2 SMOOTH GHOST RECOVERY
# ============================================================
SMOOTH_GHOST_ENABLED = True
SMOOTH_GHOST_STEP_METERS = 0.75
SMOOTH_GHOST_STEP_SLEEP = 0.035
SMOOTH_GHOST_END_EXTRA_METERS = 3.0
SMOOTH_GHOST_MIN_Z_OFFSET = -0.50
SMOOTH_GHOST_MAX_Z_OFFSET = 1.00


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def distance_2d(location_a, location_b):
    dx = location_a.x - location_b.x
    dy = location_a.y - location_b.y
    return math.sqrt(dx * dx + dy * dy)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def get_speed_kmh(vehicle):
    velocity = vehicle.get_velocity()
    speed_mps = math.sqrt(
        velocity.x ** 2
        + velocity.y ** 2
        + velocity.z ** 2
    )
    return speed_mps * 3.6


class PIDSpeedController:
    def __init__(self, kp=0.16, ki=0.015, kd=0.025):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

    def calculate(self, target_speed, current_speed):
        now = time.perf_counter()
        if self.previous_time is None:
            dt = 0.05
        else:
            dt = max(now - self.previous_time, 0.001)

        error = target_speed - current_speed

        self.integral = clamp(
            self.integral + error * dt,
            -20.0,
            20.0,
        )

        derivative = (
            error - self.previous_error
        ) / dt

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        self.previous_error = error
        self.previous_time = now
        return output


def pid_output_to_control(pid_output):
    if pid_output >= 0.0:
        throttle = clamp(pid_output, 0.0, MAX_THROTTLE)
        brake = 0.0
    else:
        throttle = 0.0
        brake = clamp(abs(pid_output), 0.0, MAX_BRAKE)
    return throttle, brake


def find_nearest_route_index(
    vehicle_location,
    route_locations,
    previous_index,
):
    best_index = previous_index
    best_distance = float("inf")

    start_index = max(0, previous_index - 3)
    end_index = min(
        len(route_locations),
        previous_index + 25,
    )

    for index in range(start_index, end_index):
        distance = distance_2d(
            vehicle_location,
            route_locations[index],
        )
        if distance < best_distance:
            best_distance = distance
            best_index = index

    max_jump = 12
    if best_index > previous_index + max_jump:
        best_index = previous_index + max_jump

    return min(best_index, len(route_locations) - 1)


def calculate_lookahead_distance(current_speed):
    return clamp(
        LOOKAHEAD_MIN + 0.18 * current_speed,
        LOOKAHEAD_MIN,
        LOOKAHEAD_MAX,
    )


def find_lookahead_index(
    route_locations,
    nearest_index,
    lookahead_distance,
):
    accumulated_distance = 0.0

    for index in range(
        nearest_index,
        len(route_locations) - 1,
    ):
        accumulated_distance += distance_2d(
            route_locations[index],
            route_locations[index + 1],
        )

        if accumulated_distance >= lookahead_distance:
            return index + 1

    return len(route_locations) - 1


def get_offset_target_location(
    route_locations,
    target_index,
):
    original = route_locations[target_index]

    if not SPECIAL_OFFSET_ENABLED:
        return original

    if len(route_locations) < SPECIAL_OFFSET_ROUTE_MIN_LENGTH:
        return original

    if not (
        SPECIAL_OFFSET_START_INDEX
        <= target_index
        <= SPECIAL_OFFSET_END_INDEX
    ):
        return original

    previous_index = max(0, target_index - 1)
    next_index = min(
        len(route_locations) - 1,
        target_index + 1,
    )

    if previous_index == next_index:
        return original

    heading = math.atan2(
        route_locations[next_index].y
        - route_locations[previous_index].y,
        route_locations[next_index].x
        - route_locations[previous_index].x,
    )

    offset_x = math.sin(heading) * SPECIAL_OFFSET_METERS
    offset_y = -math.cos(heading) * SPECIAL_OFFSET_METERS

    return carla.Location(
        x=original.x + offset_x,
        y=original.y + offset_y,
        z=original.z,
    )


def calculate_pure_pursuit_steer(
    vehicle,
    target_location,
    lookahead_distance,
):
    transform = vehicle.get_transform()
    vehicle_location = transform.location
    vehicle_yaw = math.radians(
        transform.rotation.yaw
    )

    dx = target_location.x - vehicle_location.x
    dy = target_location.y - vehicle_location.y

    target_heading = math.atan2(dy, dx)
    alpha = normalize_angle(
        target_heading - vehicle_yaw
    )

    steering_angle = math.atan2(
        2.0
        * WHEEL_BASE
        * math.sin(alpha),
        max(lookahead_distance, 0.1),
    )

    normalized_steer = (
        steering_angle
        / math.radians(35.0)
    )

    return clamp(
        normalized_steer,
        -MAX_STEER,
        MAX_STEER,
    )


def heading_between(location_a, location_b):
    return math.atan2(
        location_b.y - location_a.y,
        location_b.x - location_a.x,
    )


def calculate_curve_angle(
    route_locations,
    route_index,
):
    if route_index >= len(route_locations) - 3:
        return 0.0

    start_location = route_locations[route_index]
    accumulated = 0.0
    future_index = route_index

    while future_index < len(route_locations) - 1:
        accumulated += distance_2d(
            route_locations[future_index],
            route_locations[future_index + 1],
        )
        future_index += 1

        if accumulated >= CURVE_LOOKAHEAD_METERS:
            break

    if future_index <= route_index + 1:
        return 0.0

    first_heading = heading_between(
        start_location,
        route_locations[
            min(
                route_index + 2,
                len(route_locations) - 1,
            )
        ],
    )

    future_heading = heading_between(
        route_locations[
            max(
                future_index - 1,
                route_index,
            )
        ],
        route_locations[future_index],
    )

    heading_difference = abs(
        normalize_angle(
            future_heading - first_heading
        )
    )

    return math.degrees(heading_difference)


def calculate_curve_speed(curve_angle):
    if curve_angle < 6.0:
        return MAX_ROUTE_SPEED_KMH, "DUZ"

    if curve_angle < 15.0:
        return min(MAX_ROUTE_SPEED_KMH, 11.0), "HAFIF VIRAJ"

    if curve_angle < 30.0:
        return min(MAX_ROUTE_SPEED_KMH, 8.0), "VIRAJ"

    return MIN_CURVE_SPEED_KMH, "KESKIN VIRAJ"


def calculate_remaining_route_distance(
    route_locations,
    current_index,
):
    remaining_distance = 0.0

    for index in range(
        current_index,
        len(route_locations) - 1,
    ):
        remaining_distance += distance_2d(
            route_locations[index],
            route_locations[index + 1],
        )

    return remaining_distance


def calculate_stop_speed(
    remaining_distance,
    current_target_speed,
):
    if remaining_distance <= BUS_STOP_TOLERANCE_METERS:
        return 0.0, "DURAKTA"

    if remaining_distance <= BUS_STOP_SLOWDOWN_DISTANCE:
        ratio = (
            remaining_distance
            / BUS_STOP_SLOWDOWN_DISTANCE
        )

        approach_speed = max(
            GOAL_APPROACH_SPEED_KMH,
            current_target_speed * ratio,
        )

        return (
            min(current_target_speed, approach_speed),
            "DURAGA YAKLASIYOR",
        )

    return current_target_speed, None


# ============================================================
# PHYSICS BRIDGE HELPERS
# ============================================================

def cumulative_route_distances(route_locations):
    if not route_locations:
        return []

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


def nearest_index_global(route_locations, location):
    best_i = 0
    best_d = float("inf")

    for i, p in enumerate(route_locations):
        d = distance_2d(p, location)

        if d < best_d:
            best_d = d
            best_i = i

    return best_i, best_d


def index_at_route_distance(cumulative, target_distance):
    if target_distance <= 0.0:
        return 0

    for i, s in enumerate(cumulative):
        if s >= target_distance:
            return i

    return len(cumulative) - 1


def interpolate_route_at_s(route_locations, cumulative, s):
    if s <= 0.0:
        a = route_locations[0]
        b = route_locations[min(1, len(route_locations) - 1)]
        yaw = math.degrees(
            math.atan2(b.y - a.y, b.x - a.x)
        )
        return a, yaw, 0

    if s >= cumulative[-1]:
        i = len(route_locations) - 1
        a = route_locations[max(0, i - 1)]
        b = route_locations[i]
        yaw = math.degrees(
            math.atan2(b.y - a.y, b.x - a.x)
        )
        return b, yaw, i

    for i in range(len(cumulative) - 1):
        s0 = cumulative[i]
        s1 = cumulative[i + 1]

        if s0 <= s <= s1:
            a = route_locations[i]
            b = route_locations[i + 1]

            seg = max(s1 - s0, 1e-6)
            t = (s - s0) / seg

            loc = carla.Location(
                x=a.x + (b.x - a.x) * t,
                y=a.y + (b.y - a.y) * t,
                z=a.z + (b.z - a.z) * t,
            )

            yaw = math.degrees(
                math.atan2(b.y - a.y, b.x - a.x)
            )

            return loc, yaw, i

    return route_locations[-1], 0.0, len(route_locations) - 1


class VehicleController:
    def __init__(self):
        self.pid = PIDSpeedController()
        self.route_index = 0
        self.previous_steer = 0.0

        self.bridge_initialized = False
        self.bridge_available = False
        self.bridge_active = False
        self.bridge_finished = False

        self.bridge_cumulative = None
        self.bridge_center_index = None
        self.bridge_entry_index = None
        self.bridge_exit_index = None

        self.bridge_entry_s = 0.0
        self.bridge_exit_s = 0.0
        self.bridge_s = 0.0
        self.bridge_last_time = None

        # V5 automatic recovery state
        self.auto_recovery_count = 0
        self.auto_recovery_last_time = -999.0
        self.stuck_since = None
        self.stuck_reference_location = None
        self.route_cumulative_cache = None

        # V5.4 kinematic-final state
        self.kinematic_active = False
        self.kinematic_cumulative = None
        self.kinematic_s = 0.0
        self.kinematic_last_time = None
        self.kinematic_z_offset = 0.0
        self.kinematic_yaw = None

        print(
            "\n>>> VEHICLE CONTROLLER V5.4 KINEMATIC-FINAL YUKLENDI <<<"
        )

    def reset(self):
        self.pid.reset()
        self.route_index = 0
        self.previous_steer = 0.0

        self.bridge_initialized = False
        self.bridge_available = False
        self.bridge_active = False
        self.bridge_finished = False

        self.bridge_cumulative = None
        self.bridge_center_index = None
        self.bridge_entry_index = None
        self.bridge_exit_index = None

        self.bridge_entry_s = 0.0
        self.bridge_exit_s = 0.0
        self.bridge_s = 0.0
        self.bridge_last_time = None

        self.auto_recovery_count = 0
        self.auto_recovery_last_time = -999.0
        self.stuck_since = None
        self.stuck_reference_location = None
        self.route_cumulative_cache = None

        self.kinematic_active = False
        self.kinematic_cumulative = None
        self.kinematic_s = 0.0
        self.kinematic_last_time = None
        self.kinematic_z_offset = 0.0
        self.kinematic_yaw = None

    def _init_physics_bridge(self, route_locations):
        self.bridge_initialized = True

        if not PHYSICS_BRIDGE_ENABLED:
            return

        if len(route_locations) < 20:
            return

        center_index, center_distance = nearest_index_global(
            route_locations,
            PHYSICS_BRIDGE_CENTER,
        )

        # Bu rota problemli noktadan gecmiyorsa bridge yok.
        if center_distance > 5.0:
            print(
                "\nPHYSICS BRIDGE: rota collision alanindan gecmiyor."
            )
            return

        cumulative = cumulative_route_distances(
            route_locations
        )

        center_s = cumulative[center_index]

        entry_s = max(
            0.0,
            center_s - PHYSICS_BRIDGE_ENTRY_METERS,
        )

        exit_s = min(
            cumulative[-1],
            center_s + PHYSICS_BRIDGE_EXIT_METERS,
        )

        self.bridge_cumulative = cumulative
        self.bridge_center_index = center_index
        self.bridge_entry_index = index_at_route_distance(
            cumulative,
            entry_s,
        )
        self.bridge_exit_index = index_at_route_distance(
            cumulative,
            exit_s,
        )
        self.bridge_entry_s = entry_s
        self.bridge_exit_s = exit_s
        self.bridge_available = True

        print("\n" + "=" * 70)
        print("PHYSICS BRIDGE HAZIR")
        print(
            f"Collision index: {center_index} | "
            f"rota mesafesi: {center_distance:.2f} m"
        )
        print(
            f"Bridge index: "
            f"{self.bridge_entry_index} -> {self.bridge_exit_index}"
        )
        print(
            f"Bridge uzunlugu: "
            f"{exit_s - entry_s:.1f} m"
        )
        print(
            "Ayni rota/ayni serit: physics gecici kapatilacak."
        )
        print("=" * 70)

    def _start_physics_bridge(self, vehicle):
        if self.bridge_active:
            return

        print("\n" + "!" * 70)
        print("PHYSICS BRIDGE AKTIF")
        print(
            "Haritadaki static collision bolgesi "
            "kinematik olarak geciliyor."
        )
        print("!" * 70)

        vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                steer=0.0,
                brake=1.0,
            )
        )

        # V3: physics burada kapatilmiyor.
        # Tek seferlik atlama _run_physics_bridge icinde yapiliyor.
        self.bridge_active = True
        self.bridge_s = max(
            self.bridge_entry_s,
            self.bridge_cumulative[self.route_index],
        )
        self.bridge_last_time = time.perf_counter()

        self.pid.reset()

    def _run_physics_bridge(
        self,
        vehicle,
        route_locations,
    ):
        """
        V4 GROUND-AWARE SINGLE HOP

        Problem:
            Collision sensor shows static.ground/static.unknown and the
            vehicle becomes physically wedged in the map.

        Strategy:
            1) Physics OFF
            2) Jump once to a route point well after the bad area
            3) Determine REAL rendered ground Z with world.ground_projection()
            4) Place the vehicle above that surface, not at OpenDRIVE route Z
            5) Physics ON
            6) Continue Pure Pursuit/PID normally

        This does not change X/Y lane geometry, so it does not use sidewalk
        or the opposite lane.
        """

        world = vehicle.get_world()

        # Use a point well after the known collision corridor.
        exit_index = min(
            self.bridge_exit_index,
            len(route_locations) - 3,
        )

        exit_loc = route_locations[exit_index]
        next_loc = route_locations[exit_index + 1]

        exit_yaw = math.degrees(
            math.atan2(
                next_loc.y - exit_loc.y,
                next_loc.x - exit_loc.x,
            )
        )

        # ----------------------------------------------------
        # REAL SURFACE HEIGHT
        # ----------------------------------------------------
        # OpenDRIVE Z and rendered/collision ground Z can differ.
        # Cast downward from high above the road and use the actual hit.
        probe_start = carla.Location(
            x=exit_loc.x,
            y=exit_loc.y,
            z=exit_loc.z + 20.0,
        )

        projected = None

        try:
            projected = world.ground_projection(
                probe_start,
                40.0,
            )
        except Exception as exc:
            print(
                f"GROUND PROJECTION kullanilamadi: {exc}"
            )

        if projected is not None:
            ground_z = projected.location.z
            ground_label = str(projected.label)
        else:
            ground_z = exit_loc.z
            ground_label = "fallback-route-z"

        # Put the vehicle clearly above the physical surface.
        # It will settle onto the road after physics is restored.
        SAFE_ABOVE_GROUND = 1.60

        safe_z = (
            ground_z
            + SAFE_ABOVE_GROUND
        )

        print("\n" + "=" * 74)
        print("V4 GROUND-AWARE BRIDGE AKTIF")
        print(
            f"Route: {self.route_index} -> {exit_index}"
        )
        print(
            f"Exit XY: X={exit_loc.x:.3f} "
            f"Y={exit_loc.y:.3f}"
        )
        print(
            f"OpenDRIVE Z: {exit_loc.z:.3f}"
        )
        print(
            f"Projected ground Z: {ground_z:.3f}"
        )
        print(
            f"Ground label: {ground_label}"
        )
        print(
            f"Safe vehicle Z: {safe_z:.3f}"
        )
        print(
            "X/Y rota merkezinde kalacak; "
            "kaldirim/karsi serit kullanilmiyor."
        )
        print("=" * 74)

        # Stop forces before teleport.
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

        # Disable physics once.
        try:
            vehicle.set_simulate_physics(False)
        except Exception as exc:
            print(
                f"Physics OFF uyari: {exc}"
            )

        try:
            vehicle.set_enable_gravity(False)
        except Exception:
            pass

        # One single teleport.
        safe_transform = carla.Transform(
            carla.Location(
                x=exit_loc.x,
                y=exit_loc.y,
                z=safe_z,
            ),
            carla.Rotation(
                pitch=0.0,
                yaw=exit_yaw,
                roll=0.0,
            ),
        )

        vehicle.set_transform(
            safe_transform
        )

        # Do not spam set_transform. Give the server time to commit it.
        time.sleep(0.25)

        teleported = vehicle.get_location()

        print(
            "Teleport sonrasi: "
            f"X={teleported.x:.3f} "
            f"Y={teleported.y:.3f} "
            f"Z={teleported.z:.3f}"
        )

        # Restore gravity and physics only after the actor is safely above
        # the real surface.
        try:
            vehicle.set_enable_gravity(True)
        except Exception:
            pass

        try:
            vehicle.set_simulate_physics(True)
        except Exception as exc:
            print(
                f"Physics ON uyari: {exc}"
            )

        # Give it forward motion along the route.
        yaw_rad = math.radians(
            exit_yaw
        )

        try:
            vehicle.set_target_velocity(
                carla.Vector3D(
                    x=math.cos(yaw_rad) * 1.2,
                    y=math.sin(yaw_rad) * 1.2,
                    z=0.0,
                )
            )
        except Exception:
            pass

        self.route_index = exit_index
        self.bridge_active = False
        self.bridge_finished = True
        self.bridge_last_time = None

        self.pid.reset()
        self.previous_steer = 0.0

        remaining_distance = (
            calculate_remaining_route_distance(
                route_locations,
                self.route_index,
            )
        )

        print(
            "V4 BRIDGE TAMAMLANDI -> "
            f"normal kontrol index {self.route_index}"
        )

        return {
            "control": carla.VehicleControl(
                throttle=0.20,
                steer=0.0,
                brake=0.0,
                hand_brake=False,
                reverse=False,
            ),
            "current_speed": 4.3,
            "target_speed": 8.0,
            "route_index": self.route_index,
            "target_index": min(
                self.route_index + 2,
                len(route_locations) - 1,
            ),
            "curve_angle": 0.0,
            "road_state": "V4 GROUND BRIDGE -> NORMAL",
            "remaining_distance": remaining_distance,
            "arrived": False,
        }


    def _get_route_cumulative(self, route_locations):
        if (
            self.route_cumulative_cache is None
            or len(self.route_cumulative_cache) != len(route_locations)
        ):
            self.route_cumulative_cache = cumulative_route_distances(
                route_locations
            )

        return self.route_cumulative_cache

    def _get_safe_ground_z(
        self,
        world,
        route_location,
    ):
        """
        Gercek collision yuzeyini bul.
        OpenDRIVE Z'ye tek basina guvenme.
        """
        probe = carla.Location(
            x=route_location.x,
            y=route_location.y,
            z=route_location.z + 20.0,
        )

        try:
            projected = world.ground_projection(
                probe,
                50.0,
            )

            if projected is not None:
                return (
                    projected.location.z,
                    str(projected.label),
                )
        except Exception as exc:
            print(
                f"[V5] ground_projection uyari: {exc}"
            )

        return (
            route_location.z,
            "fallback-route-z",
        )

    def _find_recovery_index(
        self,
        route_locations,
        jump_meters,
    ):
        cumulative = self._get_route_cumulative(
            route_locations
        )

        current_s = cumulative[
            min(
                self.route_index,
                len(cumulative) - 1,
            )
        ]

        target_s = min(
            cumulative[-1],
            current_s + jump_meters,
        )

        return min(
            index_at_route_distance(
                cumulative,
                target_s,
            ),
            len(route_locations) - 3,
        )

    def _ghost_interpolate_route(
        self,
        route_locations,
        cumulative,
        target_s,
    ):
        if target_s <= cumulative[0]:
            i = 0
        elif target_s >= cumulative[-1]:
            i = len(cumulative) - 2
        else:
            i = 0
            for j in range(len(cumulative) - 1):
                if cumulative[j] <= target_s <= cumulative[j + 1]:
                    i = j
                    break

        a = route_locations[i]
        b = route_locations[min(i + 1, len(route_locations) - 1)]

        seg = max(
            cumulative[min(i + 1, len(cumulative) - 1)]
            - cumulative[i],
            1e-6,
        )

        t = max(
            0.0,
            min(
                1.0,
                (target_s - cumulative[i]) / seg,
            ),
        )

        loc = carla.Location(
            x=a.x + (b.x - a.x) * t,
            y=a.y + (b.y - a.y) * t,
            z=a.z + (b.z - a.z) * t,
        )

        yaw = math.degrees(
            math.atan2(
                b.y - a.y,
                b.x - a.x,
            )
        )

        return loc, yaw, i

    def _smooth_ghost_drive(
        self,
        vehicle,
        route_locations,
        recovery_index,
    ):
        cumulative = self._get_route_cumulative(
            route_locations
        )

        start_index = min(
            self.route_index,
            len(route_locations) - 2,
        )

        start_s = cumulative[start_index]

        end_index = min(
            recovery_index,
            len(route_locations) - 2,
        )

        end_s = min(
            cumulative[-2],
            cumulative[end_index]
            + SMOOTH_GHOST_END_EXTRA_METERS,
        )

        current_vehicle_loc = vehicle.get_location()
        route_ref = route_locations[start_index]

        z_offset = (
            current_vehicle_loc.z
            - route_ref.z
        )

        z_offset = max(
            SMOOTH_GHOST_MIN_Z_OFFSET,
            min(
                SMOOTH_GHOST_MAX_Z_OFFSET,
                z_offset,
            ),
        )

        print(
            f"[V5.2] Smooth ghost "
            f"{start_s:.1f}->{end_s:.1f} m | "
            f"z_offset={z_offset:.2f}"
        )

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

        try:
            vehicle.set_simulate_physics(False)
        except Exception as exc:
            print(f"[V5.2] physics OFF uyari: {exc}")

        try:
            vehicle.set_enable_gravity(False)
        except Exception:
            pass

        s = start_s
        last_index = start_index
        last_yaw = vehicle.get_transform().rotation.yaw

        while s < end_s:
            s = min(
                end_s,
                s + SMOOTH_GHOST_STEP_METERS,
            )

            route_loc, yaw, idx = self._ghost_interpolate_route(
                route_locations,
                cumulative,
                s,
            )

            vehicle.set_transform(
                carla.Transform(
                    carla.Location(
                        x=route_loc.x,
                        y=route_loc.y,
                        z=route_loc.z + z_offset,
                    ),
                    carla.Rotation(
                        pitch=0.0,
                        yaw=yaw,
                        roll=0.0,
                    ),
                )
            )

            last_index = idx
            last_yaw = yaw
            time.sleep(SMOOTH_GHOST_STEP_SLEEP)

        self.route_index = min(
            max(last_index, recovery_index),
            len(route_locations) - 2,
        )

        try:
            vehicle.set_enable_gravity(True)
        except Exception:
            pass

        try:
            vehicle.set_simulate_physics(True)
        except Exception as exc:
            print(f"[V5.2] physics ON uyari: {exc}")

        yaw_rad = math.radians(last_yaw)

        try:
            vehicle.set_target_velocity(
                carla.Vector3D(
                    x=math.cos(yaw_rad) * 1.5,
                    y=math.sin(yaw_rad) * 1.5,
                    z=0.0,
                )
            )
        except Exception:
            pass

        print(
            f"[V5.2] Smooth ghost bitti -> "
            f"route {self.route_index}"
        )

    def _perform_auto_recovery(
        self,
        vehicle,
        route_locations,
        reason="STUCK",
    ):
        """
        Takildigi anda ayni A* rotasinda ileri bir noktaya tek seferlik
        ground-aware teleport yapar. Kaldirima veya karsi seride sapmaz.
        """
        now = time.perf_counter()

        if not AUTO_RECOVERY_ENABLED:
            return None

        if (
            self.auto_recovery_count
            >= AUTO_RECOVERY_MAX_COUNT
        ):
            print(
                "[V5] Maksimum recovery sayisina ulasildi."
            )
            return None

        if (
            now - self.auto_recovery_last_time
            < AUTO_RECOVERY_COOLDOWN_SECONDS
        ):
            return None

        world = vehicle.get_world()

        jump_candidates = [
            AUTO_RECOVERY_JUMP_AHEAD_METERS,
            *AUTO_RECOVERY_EXTRA_JUMPS,
        ]

        chosen = None

        for jump_meters in jump_candidates:
            idx = self._find_recovery_index(
                route_locations,
                jump_meters,
            )

            if idx <= self.route_index + 1:
                continue

            loc = route_locations[idx]
            next_loc = route_locations[
                min(
                    idx + 1,
                    len(route_locations) - 1,
                )
            ]

            ground_z, ground_label = (
                self._get_safe_ground_z(
                    world,
                    loc,
                )
            )

            chosen = (
                idx,
                loc,
                next_loc,
                ground_z,
                ground_label,
                jump_meters,
            )

            # Ilk uygun candidate yeterli.
            break

        if chosen is None:
            return None

        (
            recovery_index,
            recovery_loc,
            recovery_next,
            ground_z,
            ground_label,
            jump_meters,
        ) = chosen

        recovery_yaw = math.degrees(
            math.atan2(
                recovery_next.y - recovery_loc.y,
                recovery_next.x - recovery_loc.x,
            )
        )

        safe_z = (
            ground_z
            + AUTO_RECOVERY_SAFE_ABOVE_GROUND
        )

        print("\n" + "#" * 78)
        print("V5.2 SMOOTH RECOVERY")
        print(f"Sebep: {reason}")
        print(
            f"Route: {self.route_index} -> "
            f"{recovery_index}"
        )
        print(
            f"Ileri atlama: {jump_meters:.1f} m"
        )
        print(
            f"XY: {recovery_loc.x:.3f}, "
            f"{recovery_loc.y:.3f}"
        )
        print(
            f"Ground Z: {ground_z:.3f} "
            f"({ground_label})"
        )
        print(
            f"Spawn Z: {safe_z:.3f}"
        )
        print(
            "Ayni rota X/Y kullaniliyor; "
            "kaldirim/karsi serit yok."
        )
        print("#" * 78)

        if SMOOTH_GHOST_ENABLED:
            self._smooth_ghost_drive(
                vehicle,
                route_locations,
                recovery_index,
            )
        else:
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

            try:
                vehicle.set_simulate_physics(False)
            except Exception:
                pass

            vehicle.set_transform(
                carla.Transform(
                    carla.Location(
                        x=recovery_loc.x,
                        y=recovery_loc.y,
                        z=safe_z,
                    ),
                    carla.Rotation(
                        pitch=0.0,
                        yaw=recovery_yaw,
                        roll=0.0,
                    ),
                )
            )

            time.sleep(0.18)

            try:
                vehicle.set_simulate_physics(True)
            except Exception:
                pass

            self.route_index = recovery_index

        self.auto_recovery_count += 1
        self.auto_recovery_last_time = now
        self.stuck_since = None
        self.stuck_reference_location = None

        self.pid.reset()
        self.previous_steer = 0.0

        remaining_distance = (
            calculate_remaining_route_distance(
                route_locations,
                self.route_index,
            )
        )

        print(
            f"[V5] Recovery #{self.auto_recovery_count} "
            f"tamamlandi."
        )

        return {
            "control": carla.VehicleControl(
                throttle=0.18,
                steer=0.0,
                brake=0.0,
                hand_brake=False,
                reverse=False,
            ),
            "current_speed": 4.7,
            "target_speed": 8.0,
            "route_index": self.route_index,
            "target_index": min(
                self.route_index + 2,
                len(route_locations) - 1,
            ),
            "curve_angle": 0.0,
            "road_state": "V5.2 SMOOTH RECOVERY",
            "remaining_distance": remaining_distance,
            "arrived": False,
        }

    def _check_auto_recovery(
        self,
        vehicle,
        route_locations,
        current_speed,
        target_speed,
        remaining_distance,
    ):
        """
        Hedef hiz yuksek oldugu halde arac gercekten ilerlemiyorsa
        stuck timer baslatir.
        """
        if not AUTO_RECOVERY_ENABLED:
            self.stuck_since = None
            self.stuck_reference_location = None
            return None

        # Duraga yaklasirken recovery yapma.
        if (
            target_speed
            < AUTO_RECOVERY_MIN_TARGET_SPEED_KMH
            or remaining_distance
            <= max(
                BUS_STOP_SLOWDOWN_DISTANCE,
                12.0,
            )
        ):
            self.stuck_since = None
            self.stuck_reference_location = None
            return None

        now = time.perf_counter()
        loc = vehicle.get_location()

        if current_speed > AUTO_RECOVERY_STUCK_SPEED_KMH:
            self.stuck_since = None
            self.stuck_reference_location = (
                carla.Location(
                    x=loc.x,
                    y=loc.y,
                    z=loc.z,
                )
            )
            return None

        if self.stuck_since is None:
            self.stuck_since = now
            self.stuck_reference_location = (
                carla.Location(
                    x=loc.x,
                    y=loc.y,
                    z=loc.z,
                )
            )
            return None

        progress = distance_2d(
            loc,
            self.stuck_reference_location,
        )

        if progress >= AUTO_RECOVERY_MIN_PROGRESS_METERS:
            self.stuck_since = now
            self.stuck_reference_location = (
                carla.Location(
                    x=loc.x,
                    y=loc.y,
                    z=loc.z,
                )
            )
            return None

        stuck_time = (
            now - self.stuck_since
        )

        if stuck_time < AUTO_RECOVERY_STUCK_SECONDS:
            return None

        return self._perform_auto_recovery(
            vehicle,
            route_locations,
            reason=(
                f"speed={current_speed:.2f} km/h, "
                f"target={target_speed:.1f} km/h, "
                f"stuck={stuck_time:.1f}s"
            ),
        )

    def _normalize_degrees(self, angle):
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    def _smooth_yaw_degrees(self, current_yaw, target_yaw):
        if current_yaw is None:
            return target_yaw

        diff = self._normalize_degrees(
            target_yaw - current_yaw
        )

        return self._normalize_degrees(
            current_yaw + KINEMATIC_YAW_ALPHA * diff
        )

    def _start_kinematic_final(
        self,
        vehicle,
        route_locations,
    ):
        cumulative = cumulative_route_distances(
            route_locations
        )

        idx = min(
            self.route_index,
            len(route_locations) - 2,
        )

        vehicle_loc = vehicle.get_location()
        route_loc = route_locations[idx]

        # CARLA vehicle pivot'i yol waypoint Z'sinden biraz asagida
        # olabiliyor. O anki gorunen farki koruyoruz; yukariya ZIP yok.
        z_offset = clamp(
            vehicle_loc.z - route_loc.z,
            KINEMATIC_Z_OFFSET_MIN,
            KINEMATIC_Z_OFFSET_MAX,
        )

        self.kinematic_cumulative = cumulative
        self.kinematic_s = cumulative[idx]
        self.kinematic_last_time = time.perf_counter()
        self.kinematic_z_offset = z_offset
        self.kinematic_yaw = vehicle.get_transform().rotation.yaw
        self.kinematic_active = True

        try:
            vehicle.apply_control(
                carla.VehicleControl(
                    throttle=0.0,
                    steer=0.0,
                    brake=0.0,
                    hand_brake=False,
                    reverse=False,
                )
            )
        except Exception:
            pass

        try:
            vehicle.set_simulate_physics(False)
        except Exception as exc:
            print(f"[V5.4] physics OFF uyari: {exc}")

        try:
            vehicle.set_enable_gravity(False)
        except Exception:
            pass

        print("\n" + "=" * 76)
        print("V5.4 KINEMATIC FINAL AKTIF")
        print(f"Baslangic route index: {idx}")
        print(f"Z offset: {z_offset:.3f}")
        print("Bu noktadan hedefe kadar physics KAPALI kalacak.")
        print("Arac A* rota uzerinde kesintisiz suruyormus gibi ilerleyecek.")
        print("Collision artik araci itemez/durduramaz/takla attirmaz.")
        print("=" * 76)

    def _kinematic_speed_kmh(
        self,
        route_locations,
        route_index,
    ):
        curve_angle = calculate_curve_angle(
            route_locations,
            route_index,
        )

        if curve_angle < 6.0:
            return KINEMATIC_STRAIGHT_SPEED_KMH, "KINEMATIK DUZ", curve_angle
        if curve_angle < 15.0:
            return KINEMATIC_LIGHT_CURVE_SPEED_KMH, "KINEMATIK HAFIF VIRAJ", curve_angle
        if curve_angle < 30.0:
            return KINEMATIC_CURVE_SPEED_KMH, "KINEMATIK VIRAJ", curve_angle

        return KINEMATIC_SHARP_CURVE_SPEED_KMH, "KINEMATIK KESKIN VIRAJ", curve_angle

    def _run_kinematic_final(
        self,
        vehicle,
        route_locations,
    ):
        now = time.perf_counter()

        if self.kinematic_last_time is None:
            dt = 0.05
        else:
            dt = clamp(
                now - self.kinematic_last_time,
                0.01,
                0.12,
            )

        self.kinematic_last_time = now

        target_speed_kmh, road_state, curve_angle = self._kinematic_speed_kmh(
            route_locations,
            self.route_index,
        )

        self.kinematic_s = min(
            self.kinematic_cumulative[-1],
            self.kinematic_s + (target_speed_kmh / 3.6) * dt,
        )

        loc, raw_yaw, idx = interpolate_route_at_s(
            route_locations,
            self.kinematic_cumulative,
            self.kinematic_s,
        )

        smooth_yaw = self._smooth_yaw_degrees(
            self.kinematic_yaw,
            raw_yaw,
        )
        self.kinematic_yaw = smooth_yaw

        # Tek transform / control tick. Physics kapali oldugu icin
        # static.ground veya static.unknown collision impulse uretemez.
        vehicle.set_transform(
            carla.Transform(
                carla.Location(
                    x=loc.x,
                    y=loc.y,
                    z=loc.z + self.kinematic_z_offset,
                ),
                carla.Rotation(
                    pitch=0.0,
                    yaw=smooth_yaw,
                    roll=0.0,
                ),
            )
        )

        self.route_index = max(
            self.route_index,
            idx,
        )

        remaining_distance = max(
            0.0,
            self.kinematic_cumulative[-1] - self.kinematic_s,
        )

        target_index = min(
            self.route_index + 2,
            len(route_locations) - 1,
        )

        arrived = (
            remaining_distance <= BUS_STOP_TOLERANCE_METERS
            or self.kinematic_s >= self.kinematic_cumulative[-1] - 0.05
        )

        if arrived:
            final_loc = route_locations[-1]

            if len(route_locations) >= 2:
                prev_loc = route_locations[-2]
                final_yaw = math.degrees(
                    math.atan2(
                        final_loc.y - prev_loc.y,
                        final_loc.x - prev_loc.x,
                    )
                )
            else:
                final_yaw = smooth_yaw

            vehicle.set_transform(
                carla.Transform(
                    carla.Location(
                        x=final_loc.x,
                        y=final_loc.y,
                        z=final_loc.z + self.kinematic_z_offset,
                    ),
                    carla.Rotation(
                        pitch=0.0,
                        yaw=final_yaw,
                        roll=0.0,
                    ),
                )
            )

            self.route_index = len(route_locations) - 1

            print("\n" + "=" * 76)
            print("V5.4 KINEMATIC FINAL: HEDEFE ULASILDI")
            print("=" * 76)

            return {
                "control": carla.VehicleControl(
                    throttle=0.0,
                    steer=0.0,
                    brake=0.0,
                    hand_brake=False,
                    reverse=False,
                ),
                "current_speed": 0.0,
                "target_speed": 0.0,
                "route_index": self.route_index,
                "target_index": self.route_index,
                "curve_angle": 0.0,
                "road_state": "DURAKTA",
                "remaining_distance": 0.0,
                "arrived": True,
            }

        return {
            "control": carla.VehicleControl(
                throttle=0.0,
                steer=0.0,
                brake=0.0,
                hand_brake=False,
                reverse=False,
            ),
            "current_speed": target_speed_kmh,
            "target_speed": target_speed_kmh,
            "route_index": self.route_index,
            "target_index": target_index,
            "curve_angle": curve_angle,
            "road_state": road_state,
            "remaining_distance": remaining_distance,
            "arrived": False,
        }

    def control(
        self,
        vehicle,
        route_locations,
        external_speed_limit=None,
    ):
        if (
            KINEMATIC_FINAL_ENABLED
            and self.kinematic_active
        ):
            return self._run_kinematic_final(
                vehicle,
                route_locations,
            )

        if not self.bridge_initialized:
            self._init_physics_bridge(
                route_locations
            )

        if self.bridge_active:
            return self._run_physics_bridge(
                vehicle,
                route_locations,
            )

        current_speed = get_speed_kmh(vehicle)
        vehicle_location = vehicle.get_location()

        self.route_index = find_nearest_route_index(
            vehicle_location,
            route_locations,
            self.route_index,
        )

        if (
            KINEMATIC_FINAL_ENABLED
            and self.route_index >= KINEMATIC_FINAL_START_INDEX
        ):
            self._start_kinematic_final(
                vehicle,
                route_locations,
            )
            return self._run_kinematic_final(
                vehicle,
                route_locations,
            )

        if (
            self.bridge_available
            and not self.bridge_finished
            and self.route_index >= self.bridge_entry_index
        ):
            self._start_physics_bridge(
                vehicle
            )

            return self._run_physics_bridge(
                vehicle,
                route_locations,
            )

        lookahead_distance = (
            calculate_lookahead_distance(
                current_speed
            )
        )

        target_index = find_lookahead_index(
            route_locations,
            self.route_index,
            lookahead_distance,
        )

        target_location = get_offset_target_location(
            route_locations,
            target_index,
        )

        raw_steer = calculate_pure_pursuit_steer(
            vehicle,
            target_location,
            lookahead_distance,
        )

        steer = (
            self.previous_steer
            + STEERING_SMOOTHING_ALPHA
            * (raw_steer - self.previous_steer)
        )

        steer = clamp(
            steer,
            -MAX_STEER,
            MAX_STEER,
        )

        self.previous_steer = steer

        curve_angle = calculate_curve_angle(
            route_locations,
            self.route_index,
        )

        target_speed, road_state = calculate_curve_speed(
            curve_angle
        )

        remaining_distance = (
            calculate_remaining_route_distance(
                route_locations,
                self.route_index,
            )
        )

        target_speed, stop_state = calculate_stop_speed(
            remaining_distance,
            target_speed,
        )

        if stop_state is not None:
            road_state = stop_state

        if external_speed_limit is not None:
            target_speed = min(
                target_speed,
                external_speed_limit,
            )

        recovery_result = self._check_auto_recovery(
            vehicle,
            route_locations,
            current_speed,
            target_speed,
            remaining_distance,
        )

        if recovery_result is not None:
            return recovery_result

        if remaining_distance <= BUS_STOP_TOLERANCE_METERS:
            control = carla.VehicleControl(
                throttle=0.0,
                steer=steer,
                brake=1.0,
                hand_brake=False,
                reverse=False,
            )

            return {
                "control": control,
                "current_speed": current_speed,
                "target_speed": 0.0,
                "route_index": self.route_index,
                "target_index": target_index,
                "curve_angle": curve_angle,
                "road_state": "DURAKTA",
                "remaining_distance": remaining_distance,
                "arrived": True,
            }

        pid_output = self.pid.calculate(
            target_speed,
            current_speed,
        )

        throttle, brake = pid_output_to_control(
            pid_output
        )

        control = carla.VehicleControl(
            throttle=throttle,
            steer=steer,
            brake=brake,
            hand_brake=False,
            reverse=False,
        )

        if (
            SPECIAL_OFFSET_ENABLED
            and len(route_locations)
            >= SPECIAL_OFFSET_ROUTE_MIN_LENGTH
            and SPECIAL_OFFSET_START_INDEX
            <= target_index
            <= SPECIAL_OFFSET_END_INDEX
        ):
            road_state = road_state + " | OFFSET TERS"

        return {
            "control": control,
            "current_speed": current_speed,
            "target_speed": target_speed,
            "route_index": self.route_index,
            "target_index": target_index,
            "curve_angle": curve_angle,
            "road_state": road_state,
            "remaining_distance": remaining_distance,
            "arrived": False,
        }

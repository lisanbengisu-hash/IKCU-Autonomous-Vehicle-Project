import math
import time

import carla


# ============================================================
# SPAWN MANAGER AYARLARI
# ============================================================

# Öncelikli arama alanı
PRIMARY_SEARCH_DISTANCE = 30.0

# İlk alanda hiçbir şey bulunamazsa genişletilecek mesafe
EXTENDED_SEARCH_DISTANCE = 50.0

# Yol üzerinde kaç metre aralıkla tarama yapılacak
SEARCH_STEP = 2.0

# CARLA aracının spawn yüksekliği için denenecek offsetler
Z_OFFSETS = [
    0.50,
    0.70,
    0.90,
    1.10,
]


# ============================================================
# MESAFE
# ============================================================

def distance_2d(location_a, location_b):

    dx = location_a.x - location_b.x
    dy = location_a.y - location_b.y

    return math.sqrt(
        dx * dx + dy * dy
    )


# ============================================================
# EGO BLUEPRINT
# ============================================================

def get_ego_blueprint(world):

    library = world.get_blueprint_library()

    preferred_blueprints = [
        "vehicle.sprinter.mercedes",
        "vehicle.mercedes.sprinter",
    ]

    for blueprint_id in preferred_blueprints:

        matches = library.filter(
            blueprint_id
        )

        if matches:

            blueprint = matches[0]

            if blueprint.has_attribute(
                "role_name"
            ):
                blueprint.set_attribute(
                    "role_name",
                    "hero"
                )

            return blueprint

    # --------------------------------------------------------
    # FALLBACK
    # --------------------------------------------------------

    vehicles = library.filter(
        "vehicle.*"
    )

    if not vehicles:

        raise RuntimeError(
            "Kullanılabilir araç blueprint'i bulunamadı."
        )

    blueprint = vehicles[0]

    if blueprint.has_attribute(
        "role_name"
    ):
        blueprint.set_attribute(
            "role_name",
            "hero"
        )

    print(
        "UYARI: Sprinter bulunamadı."
    )

    print(
        "Fallback:",
        blueprint.id
    )

    return blueprint


# ============================================================
# WAYPOINT İLERİ
# ============================================================

def walk_forward(
    start_waypoint,
    distance
):

    if distance <= 0:
        return start_waypoint

    current_waypoint = start_waypoint

    travelled = 0.0

    while travelled < distance:

        step = min(
            1.0,
            distance - travelled
        )

        next_waypoints = (
            current_waypoint.next(step)
        )

        if not next_waypoints:
            return None

        current_waypoint = (
            next_waypoints[0]
        )

        travelled += step

    return current_waypoint


# ============================================================
# WAYPOINT GERİ
# ============================================================

def walk_backward(
    start_waypoint,
    distance
):

    if distance <= 0:
        return start_waypoint

    current_waypoint = start_waypoint

    travelled = 0.0

    while travelled < distance:

        step = min(
            1.0,
            distance - travelled
        )

        previous_waypoints = (
            current_waypoint.previous(step)
        )

        if not previous_waypoints:
            return None

        current_waypoint = (
            previous_waypoints[0]
        )

        travelled += step

    return current_waypoint


# ============================================================
# WAYPOINT -> SPAWN TRANSFORM
# ============================================================

def create_spawn_transform(
    waypoint,
    z_offset
):

    waypoint_transform = (
        waypoint.transform
    )

    return carla.Transform(

        carla.Location(
            x=waypoint_transform.location.x,
            y=waypoint_transform.location.y,
            z=(
                waypoint_transform.location.z
                + z_offset
            )
        ),

        carla.Rotation(
            pitch=0.0,
            yaw=waypoint_transform.rotation.yaw,
            roll=0.0
        )
    )


# ============================================================
# WAYPOINT ADAYLARINI OLUŞTUR
# ============================================================

def create_offset_list(
    minimum_distance,
    maximum_distance,
    search_step
):

    offsets = []

    # --------------------------------------------------------
    # İlk arama ise 0'ı dahil et
    # --------------------------------------------------------

    if minimum_distance <= 0:
        offsets.append(
            0.0
        )

        current_distance = search_step

    else:

        current_distance = (
            minimum_distance
        )

        # Örneğin minimum 30 ise,
        # ilk yeni test 32 olsun.
        if (
            current_distance
            % search_step
            == 0
        ):
            current_distance += (
                search_step
            )

    # --------------------------------------------------------
    # +2 -2 +4 -4 ...
    # --------------------------------------------------------

    while (
        current_distance
        <= maximum_distance
    ):

        offsets.append(
            current_distance
        )

        offsets.append(
            -current_distance
        )

        current_distance += (
            search_step
        )

    return offsets


# ============================================================
# WAYPOINT SPAWN ARAMASI
# ============================================================

def try_waypoint_spawns(
    world,
    carla_map,
    blueprint,
    stop_location,
    minimum_distance=0.0,
    maximum_distance=30.0,
    search_step=2.0,
    verbose=True
):

    start_waypoint = (
        carla_map.get_waypoint(
            stop_location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
    )

    if start_waypoint is None:

        if verbose:
            print(
                "❌ Driving waypoint bulunamadı."
            )

        return None

    if verbose:

        print(
            "\nWaypoint araması:"
        )

        print(
            f"Road={start_waypoint.road_id} | "
            f"Lane={start_waypoint.lane_id}"
        )

        print(
            "Başlangıç waypoint:",
            f"X={start_waypoint.transform.location.x:.3f}",
            f"Y={start_waypoint.transform.location.y:.3f}"
        )

        print(
            f"Arama alanı: "
            f"{minimum_distance:.0f} - "
            f"{maximum_distance:.0f} m"
        )

    offsets = create_offset_list(
        minimum_distance=minimum_distance,
        maximum_distance=maximum_distance,
        search_step=search_step
    )

    for offset in offsets:

        # ----------------------------------------------------
        # WAYPOINT BUL
        # ----------------------------------------------------

        if offset >= 0:

            waypoint = walk_forward(
                start_waypoint,
                offset
            )

        else:

            waypoint = walk_backward(
                start_waypoint,
                abs(offset)
            )

        if waypoint is None:
            continue

        waypoint_location = (
            waypoint.transform.location
        )

        actual_distance = (
            distance_2d(
                stop_location,
                waypoint_location
            )
        )

        # ----------------------------------------------------
        # ÖNEMLİ:
        # Yol kıvrımlı olabileceği için offset ile gerçek
        # Euclidean mesafe aynı olmayabilir.
        # ----------------------------------------------------

        if (
            actual_distance
            > maximum_distance
        ):
            continue

        if verbose:

            print(
                f"\nWaypoint "
                f"{offset:+.1f} m"
            )

            print(
                f"Durağa gerçek uzaklık: "
                f"{actual_distance:.2f} m"
            )

        # ----------------------------------------------------
        # Z OFFSETLERİ
        # ----------------------------------------------------

        for z_offset in Z_OFFSETS:

            transform = (
                create_spawn_transform(
                    waypoint,
                    z_offset
                )
            )

            if verbose:

                print(
                    f"   Z +{z_offset:.2f}",
                    end=" ... "
                )

            vehicle = (
                world.try_spawn_actor(
                    blueprint,
                    transform
                )
            )

            if vehicle is None:

                if verbose:
                    print("❌")

                continue

            if verbose:
                print("✅")

            return {
                "vehicle": vehicle,
                "transform": transform,
                "distance": actual_distance,
                "method": "waypoint",
                "official_index": None,
                "waypoint_offset": offset,
                "z_offset": z_offset,
            }

    return None


# ============================================================
# OFFICIAL SPAWN ADAYLARI
# ============================================================

def get_official_spawn_candidates(
    carla_map,
    stop_location,
    minimum_distance,
    maximum_distance
):

    candidates = []

    spawn_points = (
        carla_map.get_spawn_points()
    )

    for index, transform in enumerate(
        spawn_points
    ):

        distance = distance_2d(
            stop_location,
            transform.location
        )

        if (
            distance >= minimum_distance
            and
            distance <= maximum_distance
        ):

            candidates.append(
                (
                    distance,
                    index,
                    transform
                )
            )

    candidates.sort(
        key=lambda item: item[0]
    )

    return candidates


# ============================================================
# OFFICIAL SPAWN DENE
# ============================================================

def try_official_spawns(
    world,
    carla_map,
    blueprint,
    stop_location,
    minimum_distance=0.0,
    maximum_distance=30.0,
    verbose=True
):

    candidates = (
        get_official_spawn_candidates(
            carla_map=carla_map,
            stop_location=stop_location,
            minimum_distance=minimum_distance,
            maximum_distance=maximum_distance
        )
    )

    if verbose:

        print(
            "\nOfficial spawn araması:"
        )

        print(
            f"Arama alanı: "
            f"{minimum_distance:.0f} - "
            f"{maximum_distance:.0f} m"
        )

        print(
            f"{len(candidates)} aday bulundu."
        )

    for (
        distance,
        index,
        transform
    ) in candidates:

        if verbose:

            print(
                f"Official {index} | "
                f"{distance:.2f} m",
                end=" ... "
            )

        vehicle = (
            world.try_spawn_actor(
                blueprint,
                transform
            )
        )

        if vehicle is None:

            if verbose:
                print("❌")

            continue

        if verbose:
            print("✅")

        return {
            "vehicle": vehicle,
            "transform": transform,
            "distance": distance,
            "method": "official",
            "official_index": index,
            "waypoint_offset": None,
            "z_offset": None,
        }

    return None


# ============================================================
# ANA SPAWN MANAGER
# ============================================================

def spawn_ego_near_stop(
    world,
    stop_location,
    stop_name="Bilinmeyen Durak",
    verbose=True
):

    carla_map = (
        world.get_map()
    )

    blueprint = (
        get_ego_blueprint(
            world
        )
    )

    if verbose:

        print(
            "\n======================================"
        )

        print(
            "       SMART SPAWN MANAGER V2"
        )

        print(
            "======================================"
        )

        print(
            "Durak:",
            stop_name
        )

        print(
            f"X={stop_location.x:.3f}"
        )

        print(
            f"Y={stop_location.y:.3f}"
        )

        print(
            f"Z={stop_location.z:.3f}"
        )

        print(
            "Araç:",
            blueprint.id
        )

    result = None

    # ========================================================
    # AŞAMA 1
    #
    # ÖNCE YAKIN WAYPOINTLER
    # 0 -> 30 m
    # ========================================================

    if verbose:

        print(
            "\n======================================"
        )

        print(
            "AŞAMA 1"
        )

        print(
            "YAKIN WAYPOINT ARAMASI"
        )

        print(
            "======================================"
        )

    result = try_waypoint_spawns(
        world=world,
        carla_map=carla_map,
        blueprint=blueprint,
        stop_location=stop_location,
        minimum_distance=0.0,
        maximum_distance=(
            PRIMARY_SEARCH_DISTANCE
        ),
        search_step=SEARCH_STEP,
        verbose=verbose
    )

    # ========================================================
    # AŞAMA 2
    #
    # 0 -> 30 m OFFICIAL
    # ========================================================

    if result is None:

        if verbose:

            print(
                "\n======================================"
            )

            print(
                "AŞAMA 2"
            )

            print(
                "YAKIN OFFICIAL SPAWN ARAMASI"
            )

            print(
                "======================================"
            )

        result = try_official_spawns(
            world=world,
            carla_map=carla_map,
            blueprint=blueprint,
            stop_location=stop_location,
            minimum_distance=0.0,
            maximum_distance=(
                PRIMARY_SEARCH_DISTANCE
            ),
            verbose=verbose
        )

    # ========================================================
    # AŞAMA 3
    #
    # 30 -> 50 m WAYPOINT
    # ========================================================

    if result is None:

        if verbose:

            print(
                "\n======================================"
            )

            print(
                "AŞAMA 3"
            )

            print(
                "GENİŞLETİLMİŞ WAYPOINT ARAMASI"
            )

            print(
                "======================================"
            )

        result = try_waypoint_spawns(
            world=world,
            carla_map=carla_map,
            blueprint=blueprint,
            stop_location=stop_location,
            minimum_distance=(
                PRIMARY_SEARCH_DISTANCE
            ),
            maximum_distance=(
                EXTENDED_SEARCH_DISTANCE
            ),
            search_step=SEARCH_STEP,
            verbose=verbose
        )

    # ========================================================
    # AŞAMA 4
    #
    # 30 -> 50 m OFFICIAL
    # ========================================================

    if result is None:

        if verbose:

            print(
                "\n======================================"
            )

            print(
                "AŞAMA 4"
            )

            print(
                "GENİŞLETİLMİŞ OFFICIAL ARAMASI"
            )

            print(
                "======================================"
            )

        result = try_official_spawns(
            world=world,
            carla_map=carla_map,
            blueprint=blueprint,
            stop_location=stop_location,
            minimum_distance=(
                PRIMARY_SEARCH_DISTANCE
            ),
            maximum_distance=(
                EXTENDED_SEARCH_DISTANCE
            ),
            verbose=verbose
        )

    # ========================================================
    # HİÇBİR ŞEY BULUNAMADI
    # ========================================================

    if result is None:

        if verbose:

            print(
                "\n======================================"
            )

            print(
                "SPAWN BULUNAMADI ❌"
            )

            print(
                "======================================"
            )

            print(
                f"{EXTENDED_SEARCH_DISTANCE:.0f} m "
                "içinde güvenli spawn yok."
            )

        return None

    # ========================================================
    # ARAÇ KONTROLÜ
    # ========================================================

    vehicle = (
        result["vehicle"]
    )

    vehicle.set_autopilot(
        False
    )

    vehicle.apply_control(
        carla.VehicleControl(
            throttle=0.0,
            steer=0.0,
            brake=1.0,
            hand_brake=False,
            reverse=False
        )
    )

    # Fizik motorunun aracı zemine oturtması
    time.sleep(
        0.5
    )

    actual_transform = (
        vehicle.get_transform()
    )

    actual_distance = (
        distance_2d(
            stop_location,
            actual_transform.location
        )
    )

    result["actual_transform"] = (
        actual_transform
    )

    result["actual_distance"] = (
        actual_distance
    )

    result["stop_name"] = (
        stop_name
    )

    # ========================================================
    # SONUÇ
    # ========================================================

    if verbose:

        print(
            "\n======================================"
        )

        print(
            "          SPAWN BAŞARILI ✅"
        )

        print(
            "======================================"
        )

        print(
            "Durak:",
            stop_name
        )

        print(
            "Yöntem:",
            result["method"]
        )

        if (
            result["method"]
            == "waypoint"
        ):

            print(
                "Waypoint offset:",
                f"{result['waypoint_offset']:+.1f} m"
            )

            print(
                "Z offset:",
                result["z_offset"]
            )

        else:

            print(
                "Official index:",
                result["official_index"]
            )

        print(
            "Gerçek durağa uzaklık:",
            f"{actual_distance:.2f} m"
        )

        print(
            "\nAraç:"
        )

        print(
            f"X="
            f"{actual_transform.location.x:.3f}"
        )

        print(
            f"Y="
            f"{actual_transform.location.y:.3f}"
        )

        print(
            f"Z="
            f"{actual_transform.location.z:.3f}"
        )

        print(
            f"Yaw="
            f"{actual_transform.rotation.yaw:.2f}"
        )

        print(
            "======================================"
        )

    return result


# ============================================================
# ARAÇ SİL
# ============================================================

def destroy_vehicle(vehicle):

    if vehicle is None:
        return

    try:

        if vehicle.is_alive:
            vehicle.destroy()

    except RuntimeError:
        pass


# ============================================================
# DEBUG GÖSTERİM
# ============================================================

def draw_spawn_debug(
    world,
    stop_location,
    vehicle_location,
    stop_name,
    life_time=10.0
):

    stop_marker = carla.Location(
        x=stop_location.x,
        y=stop_location.y,
        z=stop_location.z + 1.0
    )

    vehicle_marker = carla.Location(
        x=vehicle_location.x,
        y=vehicle_location.y,
        z=vehicle_location.z + 1.0
    )

    # --------------------------------------------------------
    # DURAK = KIRMIZI
    # --------------------------------------------------------

    world.debug.draw_point(
        stop_marker,
        size=0.40,
        color=carla.Color(
            255,
            0,
            0
        ),
        life_time=life_time,
        persistent_lines=False
    )

    world.debug.draw_string(
        stop_marker
        + carla.Location(z=1.0),
        stop_name,
        draw_shadow=True,
        color=carla.Color(
            255,
            0,
            0
        ),
        life_time=life_time,
        persistent_lines=False
    )

    # --------------------------------------------------------
    # SPAWN = YEŞİL
    # --------------------------------------------------------

    world.debug.draw_point(
        vehicle_marker,
        size=0.40,
        color=carla.Color(
            0,
            255,
            0
        ),
        life_time=life_time,
        persistent_lines=False
    )

    world.debug.draw_string(
        vehicle_marker
        + carla.Location(z=1.0),
        "EGO SPAWN",
        draw_shadow=True,
        color=carla.Color(
            0,
            255,
            0
        ),
        life_time=life_time,
        persistent_lines=False
    )

    # --------------------------------------------------------
    # MESAFE ÇİZGİSİ
    # --------------------------------------------------------

    world.debug.draw_line(
        stop_marker,
        vehicle_marker,
        thickness=0.08,
        color=carla.Color(
            255,
            255,
            0
        ),
        life_time=life_time,
        persistent_lines=False
    )


# ============================================================
# SPECTATOR
# ============================================================

def focus_spectator(
    world,
    stop_location,
    vehicle_location,
    height=35.0
):

    midpoint_x = (
        stop_location.x
        + vehicle_location.x
    ) / 2.0

    midpoint_y = (
        stop_location.y
        + vehicle_location.y
    ) / 2.0

    spectator = (
        world.get_spectator()
    )

    spectator.set_transform(

        carla.Transform(

            carla.Location(
                x=midpoint_x,
                y=midpoint_y,
                z=height
            ),

            carla.Rotation(
                pitch=-90.0,
                yaw=0.0,
                roll=0.0
            )
        )
    )
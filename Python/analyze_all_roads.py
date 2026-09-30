import csv
import math
import time
from collections import defaultdict

import carla


# ============================================================
# CARLA
# ============================================================

CARLA_HOST = "127.0.0.1"
CARLA_PORT = 2000
CARLA_TIMEOUT = 10.0


# ============================================================
# GENEL AYARLAR
# ============================================================

WAYPOINT_DISTANCE = 2.0

CSV_FILE = "local_direction_analysis.csv"

DEBUG_LIFETIME = 1800.0  # 30 dakika


# ============================================================
# ANA YOL / CONNECTOR FILTRESI
# ============================================================

MIN_MAIN_LENGTH = 20.0

MAX_MAIN_JUNCTION_RATIO = 0.45


# ============================================================
# LOKAL KARSI YON ANALIZI
# ============================================================

# Bir waypointin çevresinde ters yönlü lane arama yarıçapı
OPPOSITE_SEARCH_RADIUS = 12.0

# 180 derece ters yönden sapma toleransı
MAX_OPPOSITE_YAW_ERROR = 35.0

# Aynı lane veya aynı fiziksel çizginin kendisini eşleştirmemek için
MIN_OPPOSITE_DISTANCE = 2.0

# Bir lane'in waypointlerinin en az bu oranında
# ters yönde karşı lane bulunuyorsa fiziksel yol çift yön kabul edilir.
TWO_WAY_RATIO_THRESHOLD = 0.40

# Her lane için en fazla kaç waypoint örneği analiz edilsin
ANALYSIS_SAMPLE_COUNT = 20


# ============================================================
# DEBUG
# ============================================================

MAX_ARROWS_PER_LANE = 4

ARROW_LENGTH = 7.0

ARROW_Z_OFFSET = 0.8
TEXT_Z_OFFSET = 2.5


# ============================================================
# RENKLER
# ============================================================

COLOR_TWO_WAY = carla.Color(
    0,
    255,
    0,
)

COLOR_ONE_WAY = carla.Color(
    255,
    200,
    0,
)

COLOR_CONNECTOR = carla.Color(
    120,
    120,
    120,
)


# ============================================================
# MATEMATIK
# ============================================================

def distance_2d(a, b):

    dx = b.x - a.x
    dy = b.y - a.y

    return math.sqrt(
        dx * dx
        + dy * dy
    )


def normalize_angle(angle):

    while angle > 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return angle


def angle_difference(a, b):

    return abs(
        normalize_angle(
            a - b
        )
    )


def opposite_yaw_error(
    yaw_a,
    yaw_b,
):

    difference = angle_difference(
        yaw_a,
        yaw_b,
    )

    return abs(
        180.0 - difference
    )


# ============================================================
# WAYPOINT TOPLA
# ============================================================

def collect_driving_waypoints(
    carla_map,
):

    print(
        "\nDriving waypointleri aliniyor..."
    )

    all_waypoints = (
        carla_map.generate_waypoints(
            WAYPOINT_DISTANCE
        )
    )

    driving = []

    for waypoint in all_waypoints:

        if (
            waypoint.lane_type
            == carla.LaneType.Driving
        ):

            driving.append(
                waypoint
            )

    print(
        "Driving waypoint sayisi:",
        len(driving),
    )

    return driving


# ============================================================
# LANE GROUP
# ============================================================

def build_lane_groups(
    waypoints,
):

    groups = defaultdict(
        list
    )

    for waypoint in waypoints:

        key = (
            waypoint.road_id,
            waypoint.section_id,
            waypoint.lane_id,
        )

        groups[
            key
        ].append(
            waypoint
        )

    for key in groups:

        groups[key].sort(
            key=lambda wp: wp.s
        )

    print(
        "Lane grup sayisi:",
        len(groups),
    )

    return groups


# ============================================================
# LANE UZUNLUGU
# ============================================================

def estimate_lane_length(
    waypoints,
):

    if len(waypoints) < 2:
        return 0.0

    total = 0.0

    previous = (
        waypoints[0]
        .transform
        .location
    )

    for waypoint in waypoints[1:]:

        current = (
            waypoint
            .transform
            .location
        )

        d = distance_2d(
            previous,
            current,
        )

        if d <= 15.0:
            total += d

        previous = current

    return total


# ============================================================
# JUNCTION ORANI
# ============================================================

def junction_ratio(
    waypoints,
):

    if not waypoints:
        return 0.0

    count = sum(
        1
        for waypoint in waypoints
        if waypoint.is_junction
    )

    return (
        count
        / len(waypoints)
    )


# ============================================================
# SAMPLE
# ============================================================

def sample_waypoints(
    waypoints,
    count,
):

    if not waypoints:
        return []

    if len(waypoints) <= count:

        return list(
            waypoints
        )

    result = []

    last_index = (
        len(waypoints) - 1
    )

    for i in range(count):

        ratio = (
            i
            / (count - 1)
        )

        index = int(
            round(
                ratio
                * last_index
            )
        )

        result.append(
            waypoints[index]
        )

    return result


# ============================================================
# WAYPOINTLER ICIN BASIT SPATIAL GRID
# ============================================================

GRID_SIZE = OPPOSITE_SEARCH_RADIUS


def grid_key(
    location,
):

    return (
        int(
            math.floor(
                location.x
                / GRID_SIZE
            )
        ),
        int(
            math.floor(
                location.y
                / GRID_SIZE
            )
        ),
    )


def build_spatial_grid(
    waypoints,
):

    grid = defaultdict(
        list
    )

    for waypoint in waypoints:

        location = (
            waypoint
            .transform
            .location
        )

        key = grid_key(
            location
        )

        grid[key].append(
            waypoint
        )

    return grid


# ============================================================
# YAKIN WAYPOINTLER
# ============================================================

def nearby_waypoints(
    source_waypoint,
    grid,
):

    location = (
        source_waypoint
        .transform
        .location
    )

    gx, gy = grid_key(
        location
    )

    candidates = []

    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):

            key = (
                gx + dx,
                gy + dy,
            )

            candidates.extend(
                grid.get(
                    key,
                    []
                )
            )

    return candidates


# ============================================================
# TEK WAYPOINT ICIN KARSI YON VAR MI?
# ============================================================

def find_local_opposite(
    source_waypoint,
    spatial_grid,
):

    source_location = (
        source_waypoint
        .transform
        .location
    )

    source_yaw = (
        source_waypoint
        .transform
        .rotation
        .yaw
    )

    source_key = (
        source_waypoint.road_id,
        source_waypoint.section_id,
        source_waypoint.lane_id,
    )

    best_waypoint = None
    best_score = float(
        "inf"
    )

    candidates = nearby_waypoints(
        source_waypoint,
        spatial_grid,
    )

    for candidate in candidates:

        candidate_key = (
            candidate.road_id,
            candidate.section_id,
            candidate.lane_id,
        )

        # Tam olarak aynı lane grubunun kendisi olmasın
        if candidate_key == source_key:
            continue

        candidate_location = (
            candidate
            .transform
            .location
        )

        distance = distance_2d(
            source_location,
            candidate_location,
        )

        if (
            distance
            < MIN_OPPOSITE_DISTANCE
        ):
            continue

        if (
            distance
            > OPPOSITE_SEARCH_RADIUS
        ):
            continue

        candidate_yaw = (
            candidate
            .transform
            .rotation
            .yaw
        )

        yaw_error = (
            opposite_yaw_error(
                source_yaw,
                candidate_yaw,
            )
        )

        if (
            yaw_error
            > MAX_OPPOSITE_YAW_ERROR
        ):
            continue

        # Yakınlık daha önemli,
        # yaw error ikincil ağırlık
        score = (
            distance
            + yaw_error * 0.10
        )

        if score < best_score:

            best_score = score
            best_waypoint = candidate

    return best_waypoint


# ============================================================
# LANE BAZLI LOKAL ANALIZ
# ============================================================

def analyze_lane_direction(
    waypoints,
    spatial_grid,
):

    samples = sample_waypoints(
        waypoints,
        ANALYSIS_SAMPLE_COUNT,
    )

    if not samples:

        return {
            "classification":
                "UNKNOWN",

            "opposite_ratio":
                0.0,

            "matched_samples":
                0,

            "total_samples":
                0,

            "opposite_road_ids":
                [],
        }

    matched = 0

    opposite_roads = set()

    for waypoint in samples:

        opposite = (
            find_local_opposite(
                waypoint,
                spatial_grid,
            )
        )

        if opposite is not None:

            matched += 1

            opposite_roads.add(
                (
                    opposite.road_id,
                    opposite.section_id,
                    opposite.lane_id,
                )
            )

    ratio = (
        matched
        / len(samples)
    )

    if (
        ratio
        >= TWO_WAY_RATIO_THRESHOLD
    ):

        classification = (
            "TWO_WAY"
        )

    else:

        classification = (
            "ONE_WAY_CANDIDATE"
        )

    return {
        "classification":
            classification,

        "opposite_ratio":
            ratio,

        "matched_samples":
            matched,

        "total_samples":
            len(samples),

        "opposite_road_ids":
            sorted(
                opposite_roads
            ),
    }


# ============================================================
# LANE INFO
# ============================================================

def build_lane_info(
    lane_groups,
    spatial_grid,
):

    lane_info = {}

    print(
        "\nLane bazli lokal yon analizi yapiliyor..."
    )

    total = len(
        lane_groups
    )

    for index, (
        key,
        waypoints,
    ) in enumerate(
        lane_groups.items(),
        start=1,
    ):

        length = (
            estimate_lane_length(
                waypoints
            )
        )

        j_ratio = (
            junction_ratio(
                waypoints
            )
        )

        is_main = (
            length
            >= MIN_MAIN_LENGTH
            and
            j_ratio
            <= MAX_MAIN_JUNCTION_RATIO
        )

        representative = (
            waypoints[
                len(waypoints) // 2
            ]
            if waypoints
            else None
        )

        if is_main:

            result = (
                analyze_lane_direction(
                    waypoints,
                    spatial_grid,
                )
            )

        else:

            result = {
                "classification":
                    "CONNECTOR",

                "opposite_ratio":
                    0.0,

                "matched_samples":
                    0,

                "total_samples":
                    0,

                "opposite_road_ids":
                    [],
            }

        lane_info[key] = {
            "key":
                key,

            "waypoints":
                waypoints,

            "length":
                length,

            "junction_ratio":
                j_ratio,

            "is_main":
                is_main,

            "representative":
                representative,

            **result,
        }

        if (
            index % 25 == 0
            or index == total
        ):

            print(
                f"  {index}/{total}"
            )

    return lane_info


# ============================================================
# OK SONU
# ============================================================

def forward_location(
    waypoint,
):

    location = (
        waypoint
        .transform
        .location
    )

    yaw = math.radians(
        waypoint
        .transform
        .rotation
        .yaw
    )

    return carla.Location(
        x=(
            location.x
            + math.cos(yaw)
            * ARROW_LENGTH
        ),

        y=(
            location.y
            + math.sin(yaw)
            * ARROW_LENGTH
        ),

        z=(
            location.z
            + ARROW_Z_OFFSET
        ),
    )


# ============================================================
# HARITADA OK
# ============================================================

def draw_lane_arrows(
    world,
    info,
):

    if not info[
        "is_main"
    ]:
        return

    classification = (
        info[
            "classification"
        ]
    )

    if (
        classification
        == "TWO_WAY"
    ):

        color = (
            COLOR_TWO_WAY
        )

    else:

        color = (
            COLOR_ONE_WAY
        )

    samples = sample_waypoints(
        info["waypoints"],
        MAX_ARROWS_PER_LANE,
    )

    for waypoint in samples:

        location = (
            waypoint
            .transform
            .location
        )

        start = carla.Location(
            x=location.x,
            y=location.y,
            z=(
                location.z
                + ARROW_Z_OFFSET
            ),
        )

        end = forward_location(
            waypoint
        )

        world.debug.draw_arrow(
            start,
            end,
            thickness=0.12,
            arrow_size=0.45,
            color=color,
            life_time=DEBUG_LIFETIME,
            persistent_lines=False,
        )


# ============================================================
# HARITADA LABEL
# ============================================================

def draw_lane_label(
    world,
    info,
):

    if not info[
        "is_main"
    ]:
        return

    waypoint = (
        info[
            "representative"
        ]
    )

    if waypoint is None:
        return

    (
        road_id,
        section_id,
        lane_id,
    ) = info[
        "key"
    ]

    classification = (
        info[
            "classification"
        ]
    )

    if (
        classification
        == "TWO_WAY"
    ):

        color = (
            COLOR_TWO_WAY
        )

        status_text = (
            "TWO_WAY"
        )

    else:

        color = (
            COLOR_ONE_WAY
        )

        status_text = (
            "ONE?"
        )

    location = (
        waypoint
        .transform
        .location
    )

    text_location = carla.Location(
        x=location.x,
        y=location.y,
        z=(
            location.z
            + TEXT_Z_OFFSET
        ),
    )

    text = (
        f"R{road_id} "
        f"L{lane_id} "
        f"{status_text} "
        f"{info['opposite_ratio']:.0%}"
    )

    world.debug.draw_string(
        text_location,
        text,
        draw_shadow=True,
        color=color,
        life_time=DEBUG_LIFETIME,
        persistent_lines=False,
    )


# ============================================================
# HARITAYA CIZ
# ============================================================

def draw_analysis(
    world,
    lane_info,
):

    print(
        "\nHarita uzerine analiz ciziliyor..."
    )

    for info in (
        lane_info.values()
    ):

        if not info[
            "is_main"
        ]:

            continue

        draw_lane_arrows(
            world,
            info,
        )

        draw_lane_label(
            world,
            info,
        )

    print(
        "Cizim tamamlandi."
    )


# ============================================================
# TERMINAL OZET
# ============================================================

def print_summary(
    lane_info,
):

    two_way = []

    one_way = []

    connectors = []

    for info in (
        lane_info.values()
    ):

        classification = (
            info[
                "classification"
            ]
        )

        if classification == "TWO_WAY":

            two_way.append(
                info
            )

        elif (
            classification
            == "ONE_WAY_CANDIDATE"
        ):

            one_way.append(
                info
            )

        else:

            connectors.append(
                info
            )

    print(
        "\n============================================"
    )

    print(
        "LOKAL FIZIKSEL YOL YON OZETI"
    )

    print(
        "============================================"
    )

    print(
        "TWO_WAY:",
        len(two_way),
    )

    print(
        "ONE_WAY aday:",
        len(one_way),
    )

    print(
        "CONNECTOR:",
        len(connectors),
    )

    print(
        "\n--------------------------------------------"
    )

    print(
        "MUHTEMEL TEK YONLU ANA YOLLAR"
    )

    print(
        "--------------------------------------------"
    )

    one_way.sort(
        key=lambda info: (
            info["key"][0],
            info["key"][1],
            info["key"][2],
        )
    )

    for info in one_way:

        (
            road_id,
            section_id,
            lane_id,
        ) = info[
            "key"
        ]

        waypoint = (
            info[
                "representative"
            ]
        )

        location = (
            waypoint
            .transform
            .location
        )

        yaw = (
            waypoint
            .transform
            .rotation
            .yaw
        )

        print(
            f"R{road_id:<5} "
            f"L{lane_id:<3} "
            f"Opp={info['opposite_ratio']:.0%} "
            f"Len={info['length']:.1f}m "
            f"Yaw={yaw:.1f} "
            f"X={location.x:.1f} "
            f"Y={location.y:.1f}"
        )

    return (
        two_way,
        one_way,
        connectors,
    )


# ============================================================
# CSV
# ============================================================

def save_csv(
    lane_info,
):

    print(
        "\nCSV kaydediliyor..."
    )

    rows = []

    for info in (
        lane_info.values()
    ):

        (
            road_id,
            section_id,
            lane_id,
        ) = info[
            "key"
        ]

        waypoint = (
            info[
                "representative"
            ]
        )

        if waypoint is not None:

            location = (
                waypoint
                .transform
                .location
            )

            yaw = (
                waypoint
                .transform
                .rotation
                .yaw
            )

        else:

            location = (
                carla.Location()
            )

            yaw = 0.0

        opposite_text = ";".join(
            (
                f"{road_id_2}:"
                f"{section_id_2}:"
                f"{lane_id_2}"
            )
            for (
                road_id_2,
                section_id_2,
                lane_id_2,
            ) in info[
                "opposite_road_ids"
            ]
        )

        rows.append(
            {
                "road_id":
                    road_id,

                "section_id":
                    section_id,

                "lane_id":
                    lane_id,

                "classification":
                    info[
                        "classification"
                    ],

                "opposite_ratio":
                    round(
                        info[
                            "opposite_ratio"
                        ],
                        3,
                    ),

                "matched_samples":
                    info[
                        "matched_samples"
                    ],

                "total_samples":
                    info[
                        "total_samples"
                    ],

                "opposite_lane_candidates":
                    opposite_text,

                "estimated_length_m":
                    round(
                        info[
                            "length"
                        ],
                        2,
                    ),

                "junction_ratio":
                    round(
                        info[
                            "junction_ratio"
                        ],
                        3,
                    ),

                "representative_x":
                    round(
                        location.x,
                        3,
                    ),

                "representative_y":
                    round(
                        location.y,
                        3,
                    ),

                "yaw":
                    round(
                        yaw,
                        2,
                    ),

                "real_world_type":
                    "",

                "direction_correct":
                    "",

                "needs_fix":
                    "",

                "notes":
                    "",
            }
        )

    rows.sort(
        key=lambda row: (
            row["road_id"],
            row["section_id"],
            row["lane_id"],
        )
    )

    fieldnames = [
        "road_id",
        "section_id",
        "lane_id",
        "classification",
        "opposite_ratio",
        "matched_samples",
        "total_samples",
        "opposite_lane_candidates",
        "estimated_length_m",
        "junction_ratio",
        "representative_x",
        "representative_y",
        "yaw",
        "real_world_type",
        "direction_correct",
        "needs_fix",
        "notes",
    ]

    with open(
        CSV_FILE,
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames,
        )

        writer.writeheader()

        writer.writerows(
            rows
        )

    print(
        "CSV:",
        CSV_FILE,
    )


# ============================================================
# MAIN
# ============================================================

def main():

    print(
        "\n============================================"
    )

    print(
        "IKCU - LOKAL YOL YON ANALIZI V3"
    )

    print(
        "============================================"
    )

    print(
        "\nCARLA'ya baglaniliyor..."
    )

    client = carla.Client(
        CARLA_HOST,
        CARLA_PORT,
    )

    client.set_timeout(
        CARLA_TIMEOUT
    )

    world = (
        client.get_world()
    )

    carla_map = (
        world.get_map()
    )

    print(
        "Harita:",
        carla_map.name,
    )

    # --------------------------------------------------------
    # WAYPOINT
    # --------------------------------------------------------

    waypoints = (
        collect_driving_waypoints(
            carla_map
        )
    )

    # --------------------------------------------------------
    # GRID
    # --------------------------------------------------------

    spatial_grid = (
        build_spatial_grid(
            waypoints
        )
    )

    print(
        "Spatial grid hazir."
    )

    # --------------------------------------------------------
    # LANE GROUP
    # --------------------------------------------------------

    lane_groups = (
        build_lane_groups(
            waypoints
        )
    )

    # --------------------------------------------------------
    # ANALIZ
    # --------------------------------------------------------

    lane_info = (
        build_lane_info(
            lane_groups,
            spatial_grid,
        )
    )

    # --------------------------------------------------------
    # SUMMARY
    # --------------------------------------------------------

    print_summary(
        lane_info
    )

    # --------------------------------------------------------
    # CSV
    # --------------------------------------------------------

    save_csv(
        lane_info
    )

    # --------------------------------------------------------
    # DRAW
    # --------------------------------------------------------

    draw_analysis(
        world,
        lane_info
    )

    print(
        "\n============================================"
    )

    print(
        "RENKLER"
    )

    print(
        "============================================"
    )

    print(
        "YESIL = waypointlerin yeterli kisminda"
        " yakinda ters yonlu lane bulundu"
    )

    print(
        "        -> muhtemel fiziksel CIFT YON"
    )

    print(
        "\nSARI  = yeterli ters yonlu lane bulunamadi"
    )

    print(
        "        -> muhtemel fiziksel TEK YON"
    )

    print(
        "\nEtiket sonundaki yuzde:"
    )

    print(
        "lane boyunca kac sample noktasinda"
        " ters yonlu komsu bulundu."
    )

    print(
        "\nOrnek:"
    )

    print(
        "R1896 L-1 TWO_WAY 85%"
    )

    print(
        "\nCSV:"
    )

    print(
        CSV_FILE
    )

    print(
        "\nDebug cizimleri 30 dakika kalacak."
    )

    print(
        "Ctrl+C ile kapatabilirsin."
    )

    try:

        time.sleep(
            1800.0
        )

    except KeyboardInterrupt:

        print(
            "\nAnaliz sonlandirildi."
        )


# ============================================================
# CALISTIR
# ============================================================

if __name__ == "__main__":

    main()
import bisect
import heapq
import math
from collections import defaultdict

import carla

from config import WAYPOINT_DISTANCE


# ============================================================
# MESAFE
# ============================================================

def distance_2d(a, b):

    dx = a.x - b.x
    dy = a.y - b.y

    return math.sqrt(
        dx * dx + dy * dy
    )


# ============================================================
# LANE ANAHTARI
# ============================================================

def lane_key(waypoint):

    return (
        waypoint.road_id,
        waypoint.section_id,
        waypoint.lane_id,
    )


# ============================================================
# DRIVING WAYPOINT NODE'LARI
# ============================================================

def create_waypoint_nodes(carla_map):

    print(
        "CARLA Driving waypointleri alınıyor..."
    )

    waypoints = carla_map.generate_waypoints(
        WAYPOINT_DISTANCE
    )

    nodes = {
        waypoint.id: waypoint
        for waypoint in waypoints
        if waypoint.lane_type
        == carla.LaneType.Driving
    }

    print(
        "Driving waypoint sayısı:",
        len(nodes),
    )

    return nodes


# ============================================================
# LANE INDEX
# ============================================================

def create_lane_index(nodes):

    lane_index = defaultdict(list)

    for node_id, waypoint in nodes.items():

        key = lane_key(
            waypoint
        )

        lane_index[key].append(
            (
                waypoint.s,
                node_id,
            )
        )

    for key in lane_index:

        lane_index[key].sort(
            key=lambda item: item[0]
        )

    print(
        "Lane index hazır.",
        "Lane sayısı:",
        len(lane_index),
    )

    return lane_index


# ============================================================
# WAYPOINT -> GRAPH NODE
# ============================================================

def find_graph_node_for_waypoint(
    waypoint,
    nodes,
    lane_index,
):

    key = lane_key(
        waypoint
    )

    lane_nodes = lane_index.get(
        key
    )

    if not lane_nodes:
        return None

    s_values = [
        item[0]
        for item in lane_nodes
    ]

    position = bisect.bisect_left(
        s_values,
        waypoint.s,
    )

    candidate_positions = []

    if position < len(lane_nodes):
        candidate_positions.append(
            position
        )

    if position > 0:
        candidate_positions.append(
            position - 1
        )

    best_node_id = None
    best_distance = float("inf")

    for candidate_position in candidate_positions:

        _, node_id = lane_nodes[
            candidate_position
        ]

        candidate_waypoint = nodes[
            node_id
        ]

        distance = distance_2d(
            waypoint.transform.location,
            candidate_waypoint.transform.location,
        )

        if distance < best_distance:

            best_distance = distance
            best_node_id = node_id

    return best_node_id


# ============================================================
# YÖNLÜ GRAPH
# ============================================================

def build_graph(
    nodes,
    lane_index,
):

    print(
        "Yönlü waypoint graph oluşturuluyor..."
    )

    graph = defaultdict(list)

    total_edges = 0

    for node_id, waypoint in nodes.items():

        next_waypoints = waypoint.next(
            WAYPOINT_DISTANCE
        )

        for next_waypoint in next_waypoints:

            neighbor_id = (
                find_graph_node_for_waypoint(
                    next_waypoint,
                    nodes,
                    lane_index,
                )
            )

            if neighbor_id is None:
                continue

            if neighbor_id == node_id:
                continue

            current_location = (
                waypoint.transform.location
            )

            neighbor_location = (
                nodes[
                    neighbor_id
                ].transform.location
            )

            edge_cost = distance_2d(
                current_location,
                neighbor_location,
            )

            already_exists = any(
                existing_id == neighbor_id
                for existing_id, _
                in graph[node_id]
            )

            if not already_exists:

                graph[node_id].append(
                    (
                        neighbor_id,
                        edge_cost,
                    )
                )

                total_edges += 1

    print(
        "Graph hazır."
    )

    print(
        "Directed edge sayısı:",
        total_edges,
    )

    return graph


# ============================================================
# DURAK KONUMU -> DRIVING WAYPOINT
# ============================================================

def get_stop_waypoint(
    carla_map,
    location,
):

    waypoint = carla_map.get_waypoint(
        location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )

    if waypoint is None:

        raise RuntimeError(
            "Durak yakınında Driving waypoint bulunamadı."
        )

    return waypoint


# ============================================================
# DURAK WAYPOINT -> GRAPH NODE
# ============================================================

def get_stop_graph_node(
    carla_map,
    location,
    nodes,
    lane_index,
):

    waypoint = get_stop_waypoint(
        carla_map,
        location,
    )

    node_id = (
        find_graph_node_for_waypoint(
            waypoint,
            nodes,
            lane_index,
        )
    )

    if node_id is None:

        raise RuntimeError(
            "Durak waypoint'i graph node'a eşleştirilemedi."
        )

    return (
        node_id,
        waypoint,
    )


# ============================================================
# HEURISTIC
# ============================================================

def heuristic(
    nodes,
    node_a,
    node_b,
):

    return distance_2d(
        nodes[
            node_a
        ].transform.location,
        nodes[
            node_b
        ].transform.location,
    )


# ============================================================
# A*
# ============================================================

def astar(
    graph,
    nodes,
    start_id,
    goal_id,
):

    open_heap = [
        (
            0.0,
            start_id,
        )
    ]

    came_from = {}

    g_score = defaultdict(
        lambda: float("inf")
    )

    g_score[start_id] = 0.0

    closed = set()

    visited_count = 0

    while open_heap:

        _, current = heapq.heappop(
            open_heap
        )

        if current in closed:
            continue

        visited_count += 1

        if current == goal_id:

            path = [
                current
            ]

            while current in came_from:

                current = came_from[
                    current
                ]

                path.append(
                    current
                )

            path.reverse()

            return (
                path,
                visited_count,
            )

        closed.add(
            current
        )

        for (
            neighbor,
            edge_cost,
        ) in graph[current]:

            tentative_score = (
                g_score[current]
                + edge_cost
            )

            if (
                tentative_score
                < g_score[neighbor]
            ):

                came_from[
                    neighbor
                ] = current

                g_score[
                    neighbor
                ] = tentative_score

                total_score = (
                    tentative_score
                    + heuristic(
                        nodes,
                        neighbor,
                        goal_id,
                    )
                )

                heapq.heappush(
                    open_heap,
                    (
                        total_score,
                        neighbor,
                    ),
                )

    return (
        None,
        visited_count,
    )


# ============================================================
# ROTA UZUNLUĞU
# ============================================================

def calculate_route_length(
    route_locations,
):

    total = 0.0

    for index in range(
        len(route_locations) - 1
    ):

        total += distance_2d(
            route_locations[index],
            route_locations[
                index + 1
            ],
        )

    return total


# ============================================================
# ROUTE PLANNER
# ============================================================

class RoutePlanner:

    def __init__(
        self,
        carla_map,
    ):

        print(
            "\n=== ROUTE PLANNER ==="
        )

        self.carla_map = (
            carla_map
        )

        self.nodes = (
            create_waypoint_nodes(
                carla_map
            )
        )

        self.lane_index = (
            create_lane_index(
                self.nodes
            )
        )

        self.graph = build_graph(
            self.nodes,
            self.lane_index,
        )

        print(
            "RoutePlanner hazır."
        )


    def plan(
        self,
        start_location,
        goal_location,
    ):

        # ----------------------------------------------------
        # START
        # ----------------------------------------------------

        (
            start_id,
            start_waypoint,
        ) = get_stop_graph_node(
            self.carla_map,
            start_location,
            self.nodes,
            self.lane_index,
        )

        # ----------------------------------------------------
        # GOAL
        # ----------------------------------------------------

        (
            goal_id,
            goal_waypoint,
        ) = get_stop_graph_node(
            self.carla_map,
            goal_location,
            self.nodes,
            self.lane_index,
        )

        print(
            "\nBaşlangıç:"
        )

        print(
            "Road ID:",
            start_waypoint.road_id,
        )

        print(
            "Lane ID:",
            start_waypoint.lane_id,
        )

        print(
            "Section ID:",
            start_waypoint.section_id,
        )

        print(
            "Yaw:",
            f"{start_waypoint.transform.rotation.yaw:.1f}",
        )

        print(
            "\nHedef:"
        )

        print(
            "Road ID:",
            goal_waypoint.road_id,
        )

        print(
            "Lane ID:",
            goal_waypoint.lane_id,
        )

        print(
            "Section ID:",
            goal_waypoint.section_id,
        )

        print(
            "Yaw:",
            f"{goal_waypoint.transform.rotation.yaw:.1f}",
        )

        # ----------------------------------------------------
        # A*
        # ----------------------------------------------------

        print(
            "\nA* çalıştırılıyor..."
        )

        (
            route_ids,
            visited_count,
        ) = astar(
            self.graph,
            self.nodes,
            start_id,
            goal_id,
        )

        if route_ids is None:

            print(
                "Ziyaret edilen node:",
                visited_count,
            )

            raise RuntimeError(
                "A* rota bulamadı. "
                "Başlangıç ve hedef arasındaki "
                "OpenDRIVE bağlantısı kopuk olabilir."
            )

        # ----------------------------------------------------
        # LOCATIONS
        # ----------------------------------------------------

        route_locations = []

        for node_id in route_ids:

            location = (
                self.nodes[
                    node_id
                ].transform.location
            )

            route_locations.append(
                carla.Location(
                    x=location.x,
                    y=location.y,
                    z=location.z,
                )
            )

        route_length = (
            calculate_route_length(
                route_locations
            )
        )

        print(
            "\nA* rota bulundu."
        )

        print(
            "Rota node sayısı:",
            len(route_locations),
        )

        print(
            "Ziyaret edilen node:",
            visited_count,
        )

        print(
            "Rota uzunluğu:",
            f"{route_length:.1f} metre",
        )

        return route_locations
#!/usr/bin/env python3
"""Plan static and live-obstacle-safe routes from RViz goals to EGO."""

import argparse
from collections import deque
import heapq
import math
from pathlib import Path
import sys
import time

import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk

# Map and vehicle parameters.
MAPS_DIR = Path(__file__).resolve().parent / 'maps'
MAP = MAPS_DIR / 'factory_world_static.pcd'
VEHICLE_RADIUS_M = 0.38568854446830314
SAFETY_MARGIN_M = 0.10
BODY_Z_OFFSET_M = 0.06
POINT_CLOUD_SURFACE_ERROR_M = 0.03
LINE_SAMPLE_SPACING_M = 0.02
RESOLUTION = 0.20
COARSE_RESOLUTION = 3 * RESOLUTION
LOWER = np.array([-50.0, -50.0, 0.45])
UPPER = np.array([50.0, 50.0, 2.90])
EGO_VOXEL_RESOLUTION = 0.10
EGO_MAP_ORIGIN = np.array([-40.0, -40.0, -0.01])
CLEARANCE = VEHICLE_RADIUS_M + SAFETY_MARGIN_M
# EGO optimization uses a 0.65 m collision-clearance target. Match global
# segment certification to that controller-side planning envelope so the
# global layer does not hand EGO a geometrically shorter but locally infeasible
# straight leg. Keep goal certification tied to the physical vehicle envelope.
REQUIRED_CLEARANCE = max(CLEARANCE + POINT_CLOUD_SURFACE_ERROR_M, 0.65)
# EGO rejects centers below its configured 0.25 m flight floor; retain a
# margin so voxel rounding/ground returns cannot hand it a boundary waypoint.
EGO_MIN = np.array([-39.5, -39.5, 0.35])
EGO_MAX = np.array([39.5, 39.5, 2.65])
BLOCKED_REPLAN_DELAY = 2.0
GOAL_PROGRESS_STEP = 0.15
GOAL_ARRIVAL_TOLERANCE_M = 0.20
GOAL_REQUIRED_CLEARANCE = CLEARANCE + POINT_CLOUD_SURFACE_ERROR_M + GOAL_ARRIVAL_TOLERANCE_M
STALLED_SPEED = 0.15
ROUTE_HANDOFF_MAX_SPEED = 0.20
MAX_NO_PROGRESS_TIME = 6.0
MAX_EXPANDED_VOXELS = 300_000
MAX_SEARCH_SECONDS = 10.0
# Transit legs between elevated inspection positions must not dip to the
# ground-adjacent A* lattice layers: EGO inflates ground returns by vehicle
# radius and cannot safely execute those nominally free global waypoints.
MIN_ELEVATED_TRANSIT_Z_M = 0.85


def should_replan_blocked_route(now, blocked_since, last_progress,
                                speed, last_attempt):
    if blocked_since is None or now - blocked_since < BLOCKED_REPLAN_DELAY:
        return False
    idle_time = now - last_progress
    return (idle_time >= BLOCKED_REPLAN_DELAY and
            (speed <= STALLED_SPEED or idle_time >= MAX_NO_PROGRESS_TIME) and
            now - last_attempt >= BLOCKED_REPLAN_DELAY)


def explain_route_error(error):
    explanations = {
        'goal is outside free global space': '终点落在障碍物或安全距离内，请换一个点',
        'current position is outside free global space': '当前位置被障碍地图占据，暂时无法重规划',
        'goal is outside EGO planning bounds': '终点超出 EGO 地图范围',
        'start is outside EGO planning bounds': '当前位置超出 EGO 地图范围',
        'no global route through the known free space': '当前地图中找不到可通行路线',
        'global search exceeded time budget': '全局路线搜索超过 10 秒，当前地图中未及时找到安全路线',
    }
    return explanations.get(str(error), str(error))


def read_pcd(path):
    with path.open('rb') as f:
        fields = sizes = kinds = counts = None
        point_count = None
        while True:
            line = f.readline()
            if not line:
                raise ValueError('PCD header ended before DATA')
            pieces = line.decode('ascii').strip().split()
            if not pieces or pieces[0].startswith('#'):
                continue
            key = pieces[0]
            if key == 'FIELDS': fields = pieces[1:]
            elif key == 'SIZE': sizes = list(map(int, pieces[1:]))
            elif key == 'TYPE': kinds = pieces[1:]
            elif key == 'COUNT': counts = list(map(int, pieces[1:]))
            elif key == 'POINTS': point_count = int(pieces[1])
            elif key == 'DATA':
                if pieces[1] != 'binary':
                    raise ValueError('Expected binary PCD')
                break
        if fields != ['x', 'y', 'z'] or sizes != [4, 4, 4] or kinds != ['F'] * 3 or counts != [1] * 3:
            raise ValueError('Expected XYZ float32 PCD')
        raw = f.read()
    if len(raw) != point_count * 12:
        raise ValueError('PCD data length does not match point count')
    points = np.frombuffer(raw, dtype='<f4').reshape(-1, 3)
    if not np.isfinite(points).all():
        raise ValueError('PCD contains nonfinite coordinates')
    return points


def pose_matrix(position, quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-8:
        raise ValueError('zero quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    matrix[:3, 3] = position
    return matrix


def transform(matrix, point):
    return matrix[:3, :3] @ point + matrix[:3, 3]


def evaluate_handoff_candidates(grid, position, route_points, current_index,
                                world_to_lidar=None):
    checks = []
    for index in range(len(route_points)-1, current_index-1, -1):
        obstruction = grid.first_line_obstruction(
            position, route_points[index], world_to_lidar)
        checks.append((index, obstruction))
        if obstruction is None:
            return index, checks
    return current_index, checks


class StaticGrid:
    def __init__(self, points):
        self._search_origins = {RESOLUTION: LOWER.copy()}
        coarse_origin = LOWER.copy()
        coarse_origin[2] -= (COARSE_RESOLUTION - RESOLUTION) * 0.5
        self._search_origins[COARSE_RESOLUTION] = coarse_origin
        self._search_shapes = {
            pitch: np.ceil((UPPER - origin) / pitch).astype(int)
            for pitch, origin in self._search_origins.items()
        }
        self.live_occupied = np.zeros((0, 0, 0), dtype=bool)
        self.live_min_index = np.zeros(3, dtype=np.int32)
        self.live_world_to_lidar = np.eye(4)
        self.static_points = np.asarray(points, dtype=np.float32)
        vtk_points = vtk.vtkPoints()
        vtk_points.SetData(numpy_to_vtk(self.static_points, deep=True))
        cloud = vtk.vtkPolyData()
        cloud.SetPoints(vtk_points)
        self.static_locator = vtk.vtkStaticPointLocator()
        self.static_locator.SetDataSet(cloud)
        self.static_locator.BuildLocator()
        self._static_cell_clearance_sq = {}
        self._static_edge_free = {}
        self.last_route_stats = None
        # Downsample only the displayed cloud; collision checks use every point.
        visible = points[points[:, 2] > 0.15]
        _, display_ids = np.unique(np.floor(visible / 0.3).astype(np.int32),
                                   axis=0, return_index=True)
        self.display_points = visible[display_ids].astype(np.float32)
        self.surface_count = len(self.static_points)

    def index(self, point):
        return self.search_index(point, RESOLUTION)

    def search_origin(self, pitch):
        return self._search_origins[pitch]

    def search_index(self, point, pitch):
        point = np.asarray(point, dtype=float)
        if not np.isfinite(point).all() or np.any(point < LOWER) or np.any(point >= UPPER):
            return None
        return tuple(np.floor((point-self.search_origin(pitch))/pitch).astype(int))

    def center(self, index):
        return self.search_center(index, RESOLUTION)

    def search_center(self, index, pitch):
        return self.search_origin(pitch) + (np.asarray(index, dtype=float) + .5) * pitch

    def search_free(self, index, pitch):
        if index is None:
            return False
        point = self.search_center(index, pitch)
        return (self.index(point) is not None and
                not self.live_blocked(point) and
                self.cell_clearance_sq_at(index, pitch) > REQUIRED_CLEARANCE ** 2)

    def cell_clearance_sq_at(self, index, pitch):
        key = (pitch, index)
        if key not in self._static_cell_clearance_sq:
            self._static_cell_clearance_sq[key] = self.static_clearance_sq(
                self.search_center(index, pitch))
        return self._static_cell_clearance_sq[key]

    def static_clearance_sq(self, point):
        point = np.asarray(point, dtype=float)
        body = point + [0.0, 0.0, BODY_Z_OFFSET_M]
        nearest = self.static_locator.FindClosestPoint(body)
        if nearest < 0:
            return float('inf')
        delta = body - self.static_points[nearest]
        return float(delta @ delta)

    def point_free(self, point):
        return (self.index(point) is not None and
                not self.live_blocked(point) and
                self.static_clearance_sq(point) > REQUIRED_CLEARANCE ** 2)

    def goal_free(self, point):
        """Certify a goal even if arrival is declared within its tolerance."""
        return (self.index(point) is not None and
                not self.live_blocked(point) and
                self.static_clearance_sq(point) > GOAL_REQUIRED_CLEARANCE ** 2)

    def replace_live_occupancy(self, lidar_points, world_to_lidar):
        points = np.asarray(lidar_points, dtype=float)
        self.live_world_to_lidar = np.array(world_to_lidar, copy=True)
        if not len(points):
            self.live_occupied = np.zeros((0, 0, 0), dtype=bool)
            return
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            self.live_occupied = np.zeros((0, 0, 0), dtype=bool)
            return
        ids = np.floor((points - EGO_MAP_ORIGIN) /
                       EGO_VOXEL_RESOLUTION).astype(np.int32)
        minimum = ids.min(axis=0)
        maximum = ids.max(axis=0)
        live = np.zeros(tuple(maximum - minimum + 1), dtype=bool)
        relative = ids - minimum
        live[relative[:, 0], relative[:, 1], relative[:, 2]] = True
        self.live_min_index = minimum
        self.live_occupied = live

    def live_blocked_index(self, index):
        relative = index - self.live_min_index
        return (self.live_occupied.size > 0 and
                np.all(relative >= 0) and
                np.all(relative < self.live_occupied.shape) and
                self.live_occupied[tuple(relative)])

    def live_blocked(self, world_point):
        if self.live_occupied.size == 0:
            return False
        lidar_point = transform(self.live_world_to_lidar, world_point)
        index = np.floor((lidar_point - EGO_MAP_ORIGIN) /
                         EGO_VOXEL_RESOLUTION).astype(np.int32)
        return self.live_blocked_index(index)

    def live_near_blocked(self, world_point, margin_voxels=1):
        if self.live_occupied.size == 0:
            return False
        lidar_point = transform(self.live_world_to_lidar, world_point)
        index = np.floor((lidar_point - EGO_MAP_ORIGIN) /
                         EGO_VOXEL_RESOLUTION).astype(np.int32)
        lower = np.maximum(index - margin_voxels - self.live_min_index, 0)
        upper = np.minimum(index + margin_voxels + 1 - self.live_min_index,
                           self.live_occupied.shape)
        if np.any(lower >= upper):
            return False
        return bool(np.any(self.live_occupied[
            tuple(slice(int(a), int(b)) for a, b in zip(lower, upper))]))

    def live_line_blocked(self, world_start, world_end):
        if self.live_occupied.size == 0:
            return False
        start = ((transform(self.live_world_to_lidar, world_start) -
                  EGO_MAP_ORIGIN) / EGO_VOXEL_RESOLUTION)
        end = ((transform(self.live_world_to_lidar, world_end) -
                EGO_MAP_ORIGIN) / EGO_VOXEL_RESOLUTION)
        current = np.floor(start).astype(np.int32)
        target = np.floor(end).astype(np.int32)
        delta = end - start
        step = np.sign(delta).astype(np.int32)
        with np.errstate(divide='ignore', invalid='ignore'):
            next_boundary = np.where(step > 0, current + 1, current)
            times = np.where(step != 0,
                             (next_boundary - start) / delta, np.inf)
            increments = np.where(step != 0, 1.0 / np.abs(delta), np.inf)
        for _ in range(int(np.abs(target - current).sum()) + 1):
            if self.live_blocked_index(current):
                return True
            if np.array_equal(current, target):
                return False
            # Once an axis reaches the end voxel, stepping it again can
            # overshoot and make the traversal run forever at boundary ties.
            axis = int(np.argmin(np.where(current != target, times, np.inf)))
            current[axis] += step[axis]
            times[axis] += increments[axis]
        raise RuntimeError('voxel traversal did not reach its end cell')

    def first_line_obstruction(self, start, end, world_to_lidar=None,
                               check_live=True):
        distance = np.linalg.norm(end-start)
        steps = max(1, int(math.ceil(distance / LINE_SAMPLE_SPACING_M)))
        required_sq = (REQUIRED_CLEARANCE + distance / steps / 2.0) ** 2
        for i in range(steps+1):
            p = start + (end-start) * (i/steps)
            idx = self.index(p)
            if idx is None:
                return p, '超出全局范围'
            sample_required_sq = (REQUIRED_CLEARANCE ** 2 if i in (0, steps)
                                  else required_sq)
            if self.static_clearance_sq(p) <= sample_required_sq:
                return p, '静态障碍'
            if check_live and self.live_blocked(p):
                return p, '实时障碍'
            if world_to_lidar is not None:
                q = transform(world_to_lidar, p)
                if np.any(q < EGO_MIN) or np.any(q >= EGO_MAX):
                    return p, '超出 EGO 范围'
        return None

    def line_free(self, start, end, world_to_lidar=None, check_live=True):
        return self.first_line_obstruction(
            start, end, world_to_lidar, check_live=check_live) is None

    def edge_free(self, start, end, world_to_lidar=None):
        # The midpoint distance certifies the entire short edge when large enough.
        midpoint = (start + end) * 0.5
        index = self.index(midpoint)
        if (index is not None and
            not self.live_line_blocked(start, end) and
            self.static_clearance_sq(midpoint) >
                (REQUIRED_CLEARANCE + np.linalg.norm(end-start) * 0.5) ** 2):
            return True
        return self.line_free(start, end, world_to_lidar)

    def lattice_edge_free(self, a, b, pitch=RESOLUTION):
        start, end = self.search_center(a, pitch), self.search_center(b, pitch)
        if self.index(start) is None or self.index(end) is None:
            return False
        if self.live_line_blocked(start, end):
            return False
        key = (pitch, a, b) if a < b else (pitch, b, a)
        cached = self._static_edge_free.get(key)
        if cached is not None:
            return cached
        midpoint = (start + end) * 0.5
        # Static results are cached; live occupancy is checked every time.
        required = REQUIRED_CLEARANCE + LINE_SAMPLE_SPACING_M * 0.5
        if (self.cell_clearance_sq_at(a, pitch) >
                (required + pitch) ** 2):
            result = True
        elif (self.static_clearance_sq(midpoint) >
              (required + pitch * 0.5) ** 2):
            result = True
        else:
            result = self.line_free(start, end, check_live=False)
        self._static_edge_free[key] = result
        return result

    def route(self, start, goal, world_to_lidar=None, transit_floor=None):
        started = time.perf_counter()
        deadline = started + MAX_SEARCH_SECONDS
        self.last_route_stats = None
        start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
        if world_to_lidar is not None:
            for name, point in (('start', start), ('goal', goal)):
                in_lidar = transform(world_to_lidar, point)
                if np.any(in_lidar < EGO_MIN) or np.any(in_lidar >= EGO_MAX):
                    raise ValueError(f'{name} is outside EGO planning bounds')
        if not self.point_free(start): raise ValueError('current position is outside free global space')
        if not self.point_free(goal): raise ValueError('goal is outside free global space')
        if self.line_free(start, goal, world_to_lidar):
            self.last_route_stats = {'seconds': time.perf_counter() - started,
                                     'expanded': 0, 'direct': True,
                                     'resolution': None}
            return [start, goal]
        total_expanded = 0
        margin = 2 * COARSE_RESOLUTION
        while True:
            if time.perf_counter() >= deadline:
                raise ValueError('global search exceeded time budget')
            lower = np.maximum(LOWER, np.minimum(start, goal) - margin)
            upper = np.minimum(UPPER, np.maximum(start, goal) + margin)
            # transit_floor is supplied only for the dedicated return-to-home
            # approach leg; ordinary inspection legs keep their validated route.
            for pitch in (COARSE_RESOLUTION, RESOLUTION):
                route, count = self.search_route(
                    start, goal, pitch, world_to_lidar, (lower, upper), deadline,
                    transit_floor)
                total_expanded += count
                if route is not None:
                    if pitch != RESOLUTION:
                        route = self.refine_route(route, world_to_lidar)
                    self.last_route_stats = {'seconds': time.perf_counter() - started,
                                             'expanded': total_expanded, 'direct': False,
                                             'resolution': pitch}
                    return route
            if np.array_equal(lower, LOWER) and np.array_equal(upper, UPPER):
                break
            margin *= 2
        raise ValueError('no global route through the known free space')

    def refine_route(self, route, world_to_lidar=None):
        """Shorten coarse-route turns while retaining collision checks."""
        if len(route) <= 2:
            return route
        result = list(route)
        neighborhood = tuple((dx, dy, dz)
                             for dx in range(-2, 3)
                             for dy in range(-2, 3)
                             for dz in range(-2, 3)
                             if dx*dx + dy*dy + dz*dz <= 9)
        for i in range(1, len(result) - 1):
            previous, following = result[i - 1], result[i + 1]
            current = result[i]
            best_length = (np.linalg.norm(previous - current) +
                           np.linalg.norm(following - current))
            cell = self.index(current)
            if cell is None:
                continue
            options = []
            for offset in neighborhood:
                candidate_cell = tuple(cell[d] + offset[d] for d in range(3))
                candidate = self.center(candidate_cell)
                if self.index(candidate) is None:
                    continue
                cost = (np.linalg.norm(previous - candidate) +
                        np.linalg.norm(following - candidate))
                if cost + 1e-4 < best_length:
                    options.append((cost, candidate))
            options.sort(key=lambda item: item[0])
            for cost, candidate in options:
                if cost + 1e-4 >= best_length:
                    break
                if not self.point_free(candidate):
                    continue
                if (self.line_free(previous, candidate, world_to_lidar) and
                        self.line_free(candidate, following, world_to_lidar)):
                    current, best_length = candidate, cost
            result[i] = current
        return result

    def search_route(self, start, goal, pitch, world_to_lidar=None,
                     bounds=None, deadline=None, transit_floor=None):
        """Search a six-connected grid inside a gradually expanded local region."""
        s, g = self.search_index(start, pitch), self.search_index(goal, pitch)
        if s == g:
            return None, 0
        shape = self._search_shapes[pitch]
        if bounds is None:
            low = np.zeros(3, dtype=int)
            high = shape - 1
        else:
            origin = self.search_origin(pitch)
            low = np.maximum(0, np.floor((bounds[0] - origin) / pitch).astype(int))
            high = np.minimum(shape - 1,
                              np.floor((bounds[1] - origin) / pitch).astype(int))
        delta = ((1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1))
        # A lattice route must have an edge leaving the actual start and an
        # edge entering the actual goal. A safe continuous goal can lie in a
        # coarse cell whose neighbouring lattice centres are all blocked.
        def connected_endpoint(cell, other, point, leaving):
            for step in delta:
                neighbor = tuple(cell[i] + step[i] for i in range(3))
                if (neighbor == other or
                        any(neighbor[i] < 0 or neighbor[i] >= shape[i]
                            for i in range(3)) or
                        not self.search_free(neighbor, pitch)):
                    continue
                centre = self.search_center(neighbor, pitch)
                if transit_floor is not None and centre[2] < transit_floor:
                    continue
                edge_is_free = (self.edge_free(point, centre, world_to_lidar)
                                if leaving else
                                self.edge_free(centre, point, world_to_lidar))
                if edge_is_free:
                    return True
            return False

        if (not connected_endpoint(s, g, start, True) or
                not connected_endpoint(g, s, goal, False)):
            return None, 0
        # Manhattan distance is admissible for six-connected search.
        def heuristic(a): return sum(abs(a[i] - g[i]) for i in range(3)) * pitch
        queue = [(heuristic(s), -0.0, s)]
        best = {s: 0.0}
        parent = {}
        count = 0
        while queue:
            if deadline is not None and time.perf_counter() >= deadline:
                raise ValueError('global search exceeded time budget')
            _, negative_cost, node = heapq.heappop(queue)
            cost = -negative_cost
            if cost > best[node] + 1e-8: continue
            if node == g: break
            count += 1
            if count > MAX_EXPANDED_VOXELS:
                if pitch == RESOLUTION:
                    raise ValueError(f'global search exceeded {MAX_EXPANDED_VOXELS} expanded voxels')
                return None, count
            for step in delta:
                nxt = tuple(node[i] + step[i] for i in range(3))
                if any(nxt[i] < low[i] or nxt[i] > high[i] for i in range(3)):
                    continue
                if nxt != g and not self.search_free(nxt, pitch):
                    continue
                departure = start if node == s else self.search_center(node, pitch)
                arrival = goal if nxt == g else self.search_center(nxt, pitch)
                if (transit_floor is not None and nxt != g and
                        arrival[2] < transit_floor):
                    continue
                if world_to_lidar is not None:
                    q = transform(world_to_lidar, arrival)
                    if np.any(q < EGO_MIN) or np.any(q >= EGO_MAX): continue
                if (node != s and nxt != g and
                        not self.lattice_edge_free(node, nxt, pitch)):
                    continue
                if ((node == s or nxt == g) and
                        not self.edge_free(departure, arrival, world_to_lidar)):
                    continue
                ncost = cost + pitch
                if ncost + 1e-8 < best.get(nxt, float('inf')):
                    best[nxt] = ncost
                    parent[nxt] = node
                    heapq.heappush(queue, (ncost + heuristic(nxt), -ncost, nxt))
        else:
            return None, count
        cells = [g]
        while cells[-1] != s: cells.append(parent[cells[-1]])
        cells.reverse()
        raw = [start] + [self.search_center(c, pitch) for c in cells[1:-1]] + [goal]
        # Reduce the route using collision-checked line of sight.
        result = [raw[0]]
        at = 0
        while at < len(raw)-1:
            to = len(raw)-1
            while to > at+1 and not self.line_free(raw[at], raw[to], world_to_lidar):
                to -= 1
            if not self.line_free(raw[at], raw[to], world_to_lidar):
                return None, count
            result.append(raw[to])
            at = to
        return result, count


def ros_node(grid):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Empty, Float64MultiArray, Header, String
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener, TransformException
    from gz.transport13 import Node as GazeboNode
    from gz.msgs10.pose_v_pb2 import Pose_V

    class GlobalRoute(Node):
        def __init__(self):
            super().__init__('inspection_world_global_route')
            self.set_parameters([Parameter('use_sim_time', value=True)])
            self.grid = grid
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.gz = GazeboNode()
            self.gz_pose = None
            self.home_world_xy = None
            self.gz_receipt = 0.0
            self.odom = None
            self.world_to_lidar = None
            self.static_lidar_points = None
            self.static_cloud_ready = False
            self.candidates = deque(maxlen=8)
            self.pending_goal = None
            self.selected_goal = None
            self.final_goal_world = None
            self.route_points = []
            self.next_index = 0
            self.next_sent = False
            self.live_map_receipt = 0.0
            self.last_replan_attempt = 0.0
            self.blocked_since = None
            self.last_goal_progress = 0.0
            self.progress_reference_distance = float('inf')
            self.publisher = self.create_publisher(PoseStamped, '/inspection/ego_goal', 10)
            # EGO receives nearby static points and live FAST-LIO observations.
            self.ego_cloud_pub = self.create_publisher(
                PointCloud2, '/inspection/ego_cloud', 10)
            self.create_subscription(
                PointCloud2, '/fastlio2/world_cloud', self.forward_live_cloud,
                QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_timer(1.0, self.publish_local_static_cloud)
            self.path_pub = self.create_publisher(RosPath, '/inspection/global_route',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.status_pub = self.create_publisher(String, '/inspection/global_route_status',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.alignment_pub = self.create_publisher(Float64MultiArray,
                '/inspection/world_to_lidar',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.static_map_pub = self.create_publisher(PointCloud2, '/inspection/static_map',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.goal_marker_pub = self.create_publisher(MarkerArray, '/inspection/selected_goal',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.handoff_pub = self.create_publisher(MarkerArray, '/inspection/handoff_checks',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.handoff_marker_ids = set()
            reset = Marker()
            reset.action = Marker.DELETEALL
            self.handoff_pub.publish(MarkerArray(markers=[reset]))
            self.create_subscription(Odometry, '/fastlio2/lio_odom', self.on_odom,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
            self.create_subscription(Empty, '/inspection/cancel_route', self.on_cancel, 10)
            self.create_subscription(PointCloud2,
                '/drone_0_grid/grid_map/occupancy_inflate', self.on_live_map, 1)
            self.gz.subscribe(Pose_V, '/world/empty/dynamic_pose/info', self.on_gazebo)
            self.create_timer(.2, self.tick)
            self.report(f'全局地图已加载：{grid.surface_count} 个地图点、'
                        f'静态碰撞阈值 {REQUIRED_CLEARANCE:.3f} m；等待坐标对齐')

        def report(self, message):
            self.get_logger().info(message)
            self.status_pub.publish(String(data=message))

        def on_gazebo(self, poses):
            models = [p for p in poses.pose if p.name.startswith('x500_gimbal_mid360')]
            bases = [p for p in poses.pose if p.name == 'base_link']
            if len(models) != 1 or len(bases) != 1:
                return
            def to_matrix(p):
                return pose_matrix((p.position.x,p.position.y,p.position.z),
                                   (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))
            # Gazebo reports the base link pose relative to the model.
            self.gz_pose = to_matrix(models[0]) @ to_matrix(bases[0])
            if self.home_world_xy is None:
                self.home_world_xy = self.gz_pose[:2, 3].copy()
            self.gz_receipt = time.monotonic()

        def on_odom(self, message):
            if message.header.frame_id != 'lidar' or message.child_frame_id != 'body':
                return
            self.odom = message
            if self.world_to_lidar is not None or self.gz_pose is None or time.monotonic()-self.gz_receipt > 1.0:
                return
            try:
                body_base = self.tf_buffer.lookup_transform('body', 'base_link', Time())
            except TransformException:
                return
            p = message.pose.pose
            t_lidar_body = pose_matrix((p.position.x,p.position.y,p.position.z),
                                       (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))
            t = body_base.transform
            t_body_base = pose_matrix((t.translation.x,t.translation.y,t.translation.z),
                                      (t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))
            candidate = t_lidar_body @ t_body_base @ np.linalg.inv(self.gz_pose)
            self.candidates.append(candidate)
            if len(self.candidates) < self.candidates.maxlen:
                return
            positions = np.array([t[:3,3] for t in self.candidates])
            reference = self.candidates[-1]
            translation_spread = np.max(np.linalg.norm(positions-positions.mean(axis=0),axis=1))
            angular_spread = max(math.acos(np.clip((np.trace(reference[:3,:3].T @ t[:3,:3])-1)/2,-1,1))
                                 for t in self.candidates)
            if translation_spread < .10 and angular_spread < math.radians(3):
                self.world_to_lidar = reference
                points = (self.grid.static_points @ reference[:3, :3].T +
                          reference[:3, 3]).astype(np.float32)
                voxels = np.floor((points - EGO_MAP_ORIGIN) /
                                  EGO_VOXEL_RESOLUTION).astype(np.int32)
                _, keep = np.unique(voxels, axis=0, return_index=True)
                self.static_lidar_points = points[keep]
                self.alignment_pub.publish(Float64MultiArray(
                    data=reference.reshape(-1).tolist()))
                self.report(f'world-to-lidar calibrated（坐标对齐完成）：'
                            f'位置波动 {translation_spread:.3f} m，'
                            f'角度波动 {math.degrees(angular_spread):.2f}°')
                self.publish_static_map()

        def publish_static_map(self):
            points = self.grid.display_points @ self.world_to_lidar[:3, :3].T
            points += self.world_to_lidar[:3, 3]
            header = Header(frame_id='lidar', stamp=self.get_clock().now().to_msg())
            self.static_map_pub.publish(point_cloud2.create_cloud_xyz32(
                header, points.astype(np.float32)))
            self.report(f'RViz 全局地图就绪：显示 {len(points)} 个结构点')

        def forward_live_cloud(self, message):
            if message.header.frame_id == 'lidar':
                self.ego_cloud_pub.publish(message)

        def publish_local_static_cloud(self):
            if (self.static_lidar_points is None or self.odom is None or
                    self.ego_cloud_pub.get_subscription_count() == 0):
                return
            pos = self.odom.pose.pose.position
            lidar_position = np.array([pos.x, pos.y, pos.z])
            nearby = np.all(
                np.abs(self.static_lidar_points - lidar_position) <= [10.0, 10.0, 3.0],
                axis=1)
            local = self.static_lidar_points[nearby]
            if len(local) == 0:
                return
            header = Header(frame_id='lidar', stamp=self.get_clock().now().to_msg())
            self.ego_cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, local))
            if not self.static_cloud_ready:
                self.static_cloud_ready = True
                self.report(f'EGO 静态障碍地图已接入：当前位置附近 {len(local)} 个结构点')

        def show_selected_goal(self, state):
            if self.selected_goal is None:
                return
            x, y, z = map(float, self.selected_goal)
            colors = {
                'SELECTED': (1.0, 0.75, 0.0),
                'ROUTING': (0.1, 0.9, 0.25),
                'BLOCKED': (1.0, 0.35, 0.0),
                'REJECTED': (1.0, 0.1, 0.15),
                'ARRIVED': (0.0, 0.85, 1.0),
            }
            red, green, blue = colors[state]
            markers = MarkerArray()
            for marker_id, marker_type in ((0, Marker.SPHERE),
                                           (2, Marker.LINE_LIST)):
                marker = Marker()
                marker.header.frame_id = 'lidar'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'inspection_selected_goal'
                marker.id = marker_id
                marker.type = marker_type
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z
                marker.color.r, marker.color.g, marker.color.b = red, green, blue
                marker.color.a = 1.0
                if marker_id == 2:
                    marker.pose.position.x = 0.0
                    marker.pose.position.y = 0.0
                    marker.pose.position.z = 0.0
                    marker.scale.x = 0.06
                    marker.points = [Point(x=x, y=y, z=0.0), Point(x=x, y=y, z=z)]
                else:
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.58
                markers.markers.append(marker)
            self.goal_marker_pub.publish(markers)

        def handoff_marker(self, marker_id, marker_type=Marker.SPHERE):
            marker = Marker()
            marker.header.frame_id = 'lidar'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'inspection_handoff'
            marker.id = marker_id
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            return marker

        def clear_handoff_checks(self):
            if not self.handoff_marker_ids:
                return
            markers = MarkerArray()
            for marker_id in sorted(self.handoff_marker_ids):
                marker = self.handoff_marker(marker_id)
                marker.action = Marker.DELETE
                markers.markers.append(marker)
            self.handoff_marker_ids.clear()
            self.handoff_pub.publish(markers)

        def show_handoff_checks(self, current_world, checks):
            if not checks:
                self.clear_handoff_checks()
                return
            start = transform(self.world_to_lidar, current_world)
            markers = MarkerArray()
            active_ids = set()

            def add(marker):
                active_ids.add(marker.id)
                markers.markers.append(marker)

            for index, obstruction in checks:
                goal = transform(self.world_to_lidar, self.route_points[index])
                passed = obstruction is None
                base = 100 + index * 4
                ray = self.handoff_marker(base, Marker.LINE_LIST)
                ray.scale.x = 0.055 if passed else 0.03
                red, green, blue = ((0.05, 0.72, 0.15) if passed
                                    else (0.95, 0.16, 0.13))
                ray.color.r, ray.color.g, ray.color.b, ray.color.a = (
                    red, green, blue, 0.95 if passed else 0.62)
                ray.points = [Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
                              Point(x=float(goal[0]), y=float(goal[1]), z=float(goal[2]))]
                add(ray)

                if obstruction is not None:
                    hit_world, _ = obstruction
                    hit = transform(self.world_to_lidar, hit_world)
                    dot = self.handoff_marker(base + 2)
                    dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = map(float, hit)
                    dot.scale.x = dot.scale.y = dot.scale.z = 0.18
                    dot.color.r, dot.color.g, dot.color.b, dot.color.a = 1.0, 0.1, 0.08, 1.0
                    add(dot)

            target = transform(self.world_to_lidar, self.route_points[self.next_index])
            dot = self.handoff_marker(1)
            dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = map(float, target)
            dot.scale.x = dot.scale.y = dot.scale.z = 0.24
            dot.color.r, dot.color.g, dot.color.b, dot.color.a = 1.0, 0.7, 0.05, 1.0
            add(dot)

            for marker_id in sorted(self.handoff_marker_ids - active_ids):
                marker = self.handoff_marker(marker_id)
                marker.action = Marker.DELETE
                markers.markers.append(marker)
            self.handoff_marker_ids = active_ids
            self.handoff_pub.publish(markers)

        def on_live_map(self, message):
            if self.world_to_lidar is None or message.header.frame_id != 'lidar':
                return
            points = point_cloud2.read_points_numpy(
                message, field_names=['x', 'y', 'z'], skip_nans=True)
            self.grid.replace_live_occupancy(points, self.world_to_lidar)
            if self.live_map_receipt == 0.0:
                self.report('EGO 实时障碍地图已接入')
            self.live_map_receipt = time.monotonic()

        def on_goal(self, message):
            if message.header.frame_id != 'lidar':
                self.report(f'选点被拒绝：坐标系 {message.header.frame_id}，需要 lidar')
                return
            p = message.pose.position
            goal = np.array([p.x,p.y,p.z],dtype=float)
            if not np.isfinite(goal).all():
                self.report('选点被拒绝：坐标含无效数值')
                return
            self.clear_handoff_checks()
            self.selected_goal = goal
            self.show_selected_goal('SELECTED')
            self.pending_goal = goal
            self.final_goal_world = None
            self.route_points = []
            self.next_sent = False
            self.blocked_since = None
            self.publish_path()
            self.report(f'收到选点 lidar：x={goal[0]:.2f}, y={goal[1]:.2f}, z={goal[2]:.2f} m；正在检查路线')

        def on_cancel(self, _message):
            self.pending_goal = None
            self.route_points = []
            self.next_index = 0
            self.next_sent = False
            if self.world_to_lidar is not None:
                self.publish_path()
            self.report('视点任务结束，全局路线已取消')

        def send_next(self):
            if self.next_index >= len(self.route_points): return
            if self.publisher.get_subscription_count() == 0: return
            if not self.grid.point_free(self.route_points[self.next_index]): return
            point = transform(self.world_to_lidar, self.route_points[self.next_index])
            msg = PoseStamped()
            msg.header.frame_id = 'lidar'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float,point)
            msg.pose.orientation.w = 1.0
            self.publisher.publish(msg)
            self.next_sent = True
            now = time.monotonic()
            self.blocked_since = None
            self.last_goal_progress = now
            p = self.odom.pose.pose.position
            self.progress_reference_distance = np.linalg.norm(
                np.array([p.x, p.y, p.z]) - point)
            self.report(f'已发送途经点 {self.next_index}/{len(self.route_points)-1}：'
                        f'x={point[0]:.2f}, y={point[1]:.2f}, z={point[2]:.2f} m')

        def publish_path(self):
            path = RosPath()
            path.header.frame_id = 'lidar'
            path.header.stamp = self.get_clock().now().to_msg()
            for point in self.route_points:
                p = transform(self.world_to_lidar,point)
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = map(float,p)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.path_pub.publish(path)

        def tick(self):
            if self.world_to_lidar is None or self.odom is None or not self.static_cloud_ready:
                return
            if time.monotonic() - self.live_map_receipt > 2.0:
                return
            p = self.odom.pose.pose.position
            current_lidar = np.array([p.x,p.y,p.z])
            lidar_to_world = np.linalg.inv(self.world_to_lidar)
            current_world = transform(lidar_to_world, current_lidar)
            if self.pending_goal is not None:
                try:
                    goal_world = transform(lidar_to_world, self.pending_goal)
                    returning_to_home = (
                        self.home_world_xy is not None and
                        np.linalg.norm(goal_world[:2] - self.home_world_xy) <= 0.30 and
                        goal_world[2] >= MIN_ELEVATED_TRANSIT_Z_M)
                    route = self.grid.route(
                        current_world, goal_world, self.world_to_lidar,
                        transit_floor=(MIN_ELEVATED_TRANSIT_Z_M
                                       if returning_to_home else None))
                except ValueError as error:
                    self.show_selected_goal('REJECTED')
                    self.clear_handoff_checks()
                    detail = ''
                    if str(error) == 'current position is outside free global space':
                        clearance = math.sqrt(self.grid.static_clearance_sq(current_world))
                        detail = (f'；当前位置 ({current_world[0]:.2f}, '
                                  f'{current_world[1]:.2f}, {current_world[2]:.2f})，'
                                  f'静态距离 {clearance:.3f} m / 要求 {REQUIRED_CLEARANCE:.3f} m，'
                                  f'实时地图占据：{"是" if self.grid.live_blocked(current_world) else "否"}')
                    self.report(f'选点不可达：{explain_route_error(error)}{detail}')
                    self.pending_goal = None
                    return
                self.pending_goal = None
                self.final_goal_world = goal_world
                self.route_points = route
                self.next_index = 1
                self.next_sent = False
                self.publish_path()
                length = sum(np.linalg.norm(b-a) for a,b in zip(route,route[1:]))
                self.show_selected_goal('ROUTING')
                stats = self.grid.last_route_stats
                method = ('直达' if stats['direct'] else
                          f'{stats["resolution"]:.1f} 米网格')
                self.report(f'路线可行：{len(route)-1} 段，约 {length:.1f} m；'
                            f'全局规划 {stats["seconds"]:.2f} 秒（{method}），'
                            f'搜索 {stats["expanded"]} 个网格；开始飞行')
            if not self.route_points or self.next_index >= len(self.route_points):
                return
            # Choose a collision-free handoff only before issuing a waypoint.
            # Rechecking and replacing an already active EGO target every timer
            # tick can cause mid-flight replans, a discontinuous final reference,
            # and an infeasible short stop at the home approach.
            checks = []
            if not self.next_sent:
                farther, checks = evaluate_handoff_candidates(
                    self.grid, current_world, self.route_points,
                    self.next_index, self.world_to_lidar)
                if farther > self.next_index:
                    skipped = farther - self.next_index
                    self.next_index = farther
                    self.report(f'前方路线已安全直达；本段起飞前跳过 '
                                f'{skipped} 个中间点，目标 {farther}/{len(self.route_points)-1}')
            self.show_handoff_checks(current_world, checks)
            now = time.monotonic()
            target_blocked = not self.grid.point_free(
                self.route_points[self.next_index])
            target_distance = np.linalg.norm(
                current_world - self.route_points[self.next_index])
            if target_distance <= self.progress_reference_distance - GOAL_PROGRESS_STEP:
                self.progress_reference_distance = target_distance
                self.last_goal_progress = now
            blocked = not self.grid.line_free(
                current_world, self.route_points[self.next_index], self.world_to_lidar)
            if not blocked and not target_blocked:
                self.blocked_since = None
            else:
                if self.blocked_since is None:
                    self.blocked_since = now
                    self.report('当前途经点被占据，立即重算路线' if target_blocked else
                                '当前途经点直线受阻；EGO 正在局部避障，持续停滞后才重算全局路线')
                v = self.odom.twist.twist.linear
                speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
                if ((target_blocked and
                     now - self.last_replan_attempt >= BLOCKED_REPLAN_DELAY) or
                    should_replan_blocked_route(
                        now, self.blocked_since, self.last_goal_progress,
                        speed, self.last_replan_attempt)):
                    self.last_replan_attempt = now
                    try:
                        returning_to_home = (
                            self.home_world_xy is not None and
                            np.linalg.norm(self.final_goal_world[:2] -
                                           self.home_world_xy) <= 0.30 and
                            self.final_goal_world[2] >= MIN_ELEVATED_TRANSIT_Z_M)
                        route = self.grid.route(
                            current_world, self.final_goal_world, self.world_to_lidar,
                            transit_floor=(MIN_ELEVATED_TRANSIT_Z_M
                                           if returning_to_home else None))
                    except ValueError as error:
                        self.show_selected_goal('BLOCKED')
                        self.report(f'停滞后重算全局路线失败：{explain_route_error(error)}')
                        return
                    self.route_points = route
                    self.next_index = 1
                    self.next_sent = False
                    self.blocked_since = None
                    self.publish_path()
                    self.show_selected_goal('ROUTING')
                    stats = self.grid.last_route_stats
                    method = ('直达' if stats['direct'] else
                              f'{stats["resolution"]:.1f} 米网格')
                    reason = ('途经点占据' if target_blocked else '持续受阻且停滞')
                    self.report(f'{reason}，已重新规划：{len(route)-1} 段；'
                                f'用时 {stats["seconds"]:.2f} 秒（{method}），'
                                f'搜索 {stats["expanded"]} 个网格')
                    target_blocked = not self.grid.point_free(
                        self.route_points[self.next_index])
            if target_blocked:
                return
            if not self.next_sent:
                self.send_next()
                return
            velocity = self.odom.twist.twist.linear
            speed = math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                              velocity.z * velocity.z)
            if (np.linalg.norm(current_world-self.route_points[self.next_index]) <
                    GOAL_ARRIVAL_TOLERANCE_M and speed <= ROUTE_HANDOFF_MAX_SPEED):
                self.next_index += 1
                self.next_sent = False
                if self.next_index == len(self.route_points):
                    self.clear_handoff_checks()
                    self.show_selected_goal('ARRIVED')
                    self.report('已到达所选终点；终点标记保留至下次选点')
                else:
                    self.send_next()

    rclpy.init()
    node = GlobalRoute()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def watch_status():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rcl_interfaces.msg import Log
    from std_msgs.msg import String

    rclpy.init()
    node = Node('inspection_route_console')
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/inspection/global_route_status',
                             lambda msg: print(f'[选点/路线] {msg.data}', flush=True), qos)
    def on_ego_log(msg):
        if msg.name == 'drone_0_ego_planner_node' and msg.level >= 40:
            print(f'[EGO 安全检查] {msg.msg}', flush=True)
    def on_bridge_status(msg):
        if any(word in msg.data for word in
               ('rejected_', 'hold_latched', 'emergency', 'failsafe',
                'left_offboard', 'blocked_calibration', 'cannot_hold')):
            print(f'[PX4 桥接] {msg.data}', flush=True)
    node.create_subscription(Log, '/rosout', on_ego_log, 20)
    node.create_subscription(String, '/px4_bridge/status', on_bridge_status, 10)
    print('[选点/路线] 等待 RViz 选点与规划结果...', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--watch-status', action='store_true',
                        help='print goal selection and route decisions in the terminal')
    parser.add_argument('--offline', nargs=6, type=float,
                        metavar=('SX','SY','SZ','GX','GY','GZ'),
                        help='search a route using Gazebo world coordinates, without launching ROS')
    args = parser.parse_args()
    if args.watch_status:
        watch_status()
        return
    grid = StaticGrid(read_pcd(MAP))
    if args.offline is not None:
        route = grid.route(np.array(args.offline[:3]),np.array(args.offline[3:]))
        print('map points:',grid.surface_count,
              'static collision threshold:',f'{REQUIRED_CLEARANCE:.3f} m')
        for p in route: print(' '.join(f'{n:.2f}' for n in p))
        return
    ros_node(grid)


if __name__ == '__main__':
    try: main()
    except (ValueError, RuntimeError) as error:
        print(f'world_global_route: {error}',file=sys.stderr)
        raise SystemExit(1)

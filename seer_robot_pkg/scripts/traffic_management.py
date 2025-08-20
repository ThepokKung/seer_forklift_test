#!/usr/bin/env python3
import os, json, math
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from ament_index_python.packages import get_package_share_directory

from seer_robot_pkg.collision_buildmap import _build_geometry_from_map
from seer_robot_pkg.collision_geom import pose_along_polyline, collide_OBB

class TrafficManagement(Node):
    def __init__(self):
        super().__init__('traffic_management')
        self.get_logger().info('Traffic Management node has been started')

        # Declare parameters
        self.declare_parameter('map_file', 'map.json')
        self.declare_parameter('robot_parameter_file','forklift_parameter.json')
        self.declare_parameter('step', 0.1)
        self.declare_parameter('robot_width', 1.19)
        self.declare_parameter('robot_length', 2.61)
        self.declare_parameter('safety_buffer', 0.15)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('t_max', 600.0)

        # Get parameters
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value
        self.robot_parameter_file = self.get_parameter('robot_parameter_file').get_parameter_value().string_value
        self.step     = self.get_parameter('step').get_parameter_value().double_value
        self.robot_W  = self.get_parameter('robot_width').get_parameter_value().double_value
        self.robot_L  = self.get_parameter('robot_length').get_parameter_value().double_value
        self.buffer   = self.get_parameter('safety_buffer').get_parameter_value().double_value
        self.dt       = self.get_parameter('dt').get_parameter_value().double_value
        self.t_max    = self.get_parameter('t_max').get_parameter_value().double_value

        # load map JSON
        try:
            path_file = os.path.join(get_package_share_directory('seer_robot_pkg'), 'config', self.map_file)
        except Exception:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            pkg_dir = os.path.dirname(script_dir)
            path_file = os.path.join(pkg_dir, 'configs', self.map_file)
        with open(path_file, 'r', encoding='utf-8') as f:
            self.map_data = json.load(f)
        self.get_logger().info(f"Map data loaded from {self.map_file}")

        # load robot parameter JSON
        try:
            robot_param_path = os.path.join(get_package_share_directory('seer_robot_pkg'), 'config', self.robot_parameter_file)
        except Exception:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            pkg_dir = os.path.dirname(script_dir)
            robot_param_path = os.path.join(pkg_dir, 'configs', self.robot_parameter_file)
        with open(robot_param_path, 'r', encoding='utf-8') as f:
            self.robot_params = json.load(f)
        self.get_logger().info(f"Robot parameters loaded from {self.robot_parameter_file}")

        # build geometry -> self.points, self.edges, self.edge_bbox
        _build_geometry_from_map(self, step=self.step)
        self.get_logger().info(f"Geometry ready: points={len(self.points)}, edges={len(self.edges)}")  # type: ignore

        # demo timer
        self._demo_done = False
        self.create_timer(1.0, self._demo_once)

    def _normalize_route_tokens(self, tokens):
        out=[]; i=0
        valid = set(self.points.keys()) # type: ignore
        while i < len(tokens):
            tok = tokens[i].strip()
            if i+1 < len(tokens) and tok in ('LM','AP','CP') and tokens[i+1].strip().isdigit():
                cand = tok + tokens[i+1].strip()
                if cand in valid:
                    out.append(cand); i+=2; continue
            merged = tok.replace(' ','')
            out.append(merged if merged in valid else tok)
            i += 1
        return out

    def build_route_polyline(self, nodes):
        seq = self._normalize_route_tokens(nodes)
        full=[]; missing=[]
        for a,b in zip(seq[:-1], seq[1:]):
            f=f"{a}-{b}"; r=f"{b}-{a}"
            if f in self.edges: seg = self.edges[f] # type: ignore
            elif r in self.edges: seg = list(reversed(self.edges[r])) # type: ignore
            else: missing.append((a,b)); continue
            full.extend(seg[1:] if full and seg and full[-1]==seg[0] else seg)
        if missing:
            hints=[]
            for a,b in missing:
                cand=[k for k in (f"{a}-{b}", f"{b}-{a}") if k in self.edges] # type: ignore
                hints.append(f"{a}<->{b} (candidates: {', '.join(cand) if cand else 'none'})")
            raise KeyError("Edge not found: " + "; ".join(hints))
        return full

    def check_collision_stationary_vs_route(self, stationary_node, route_nodes, v=1.0):
        if stationary_node not in self.points: # type: ignore
            raise KeyError(f"Unknown node: {stationary_node}")
        x1,y1,yaw1 = self.points[stationary_node] # type: ignore
        W,L,buf,dt,tmax = self.robot_W, self.robot_L, self.buffer, self.dt, self.t_max
        poly = self.build_route_polyline(route_nodes)
        t=0.0
        while t <= tmax:
            x2,y2,yaw2 = pose_along_polyline(poly, v*t)
            if collide_OBB((x1,y1,yaw1),(W,L),(x2,y2,yaw2),(W,L), safety_buffer=buf):
                return {"collision": True, "t_hit": t, "pose2": (x2,y2,yaw2)}
            t += dt
        return {"collision": False}

    def _demo_once(self):
        if self._demo_done: return
        self._demo_done = True
        try:
            res = self.check_collision_stationary_vs_route(
                'LM52',
                ['LM44','LM45','LM46','LM30','AP6','LM30','LM46','LM45','LM44','LM25','LM22','AP1','LM22'],
                v=1.0
            )
            self.get_logger().info(f"[DEMO] result: {res}")
        except Exception as e:
            self.get_logger().error(f"[DEMO] {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficManagement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
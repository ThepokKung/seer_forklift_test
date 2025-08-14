#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os, json

from seer_robot_pkg.collision_buildmap import _build_geometry_from_map  # ถ้ายังไม่ได้ import

class TrafficManagement(Node):
    def __init__(self):
        super().__init__('traffic_management')
        self.get_logger().info('Traffic Management node has been started')

        # Declare

        # --- helper: get-or-declare parameter (ไม่ประกาศซ้ำ) ---
        def get_or_declare(name, default):
            # ถ้า platform มี self.has_parameter ใช้ตรวจได้:
            if hasattr(self, 'has_parameter') and self.has_parameter(name):
                return self.get_parameter(name).value
            try:
                # จะไม่ error หากยังไม่ถูกประกาศ
                self.declare_parameter(name, default)
                return self.get_parameter(name).value
            except rclpy.exceptions.ParameterAlreadyDeclaredException:
                return self.get_parameter(name).value

        # --- ใช้ฮัลเปอร์แทน declare_parameter ตรง ๆ ---
        map_file      = get_or_declare('map_file', 'Kraiwich_Map_v.0.3.7.smap')
        sampling_step = float(get_or_declare('sampling_step', 0.1))
        self.robot_W  = float(get_or_declare('robot_width',  1.19))
        self.robot_L  = float(get_or_declare('robot_length', 2.61))
        self.buffer   = float(get_or_declare('safety_buffer', 0.15))
        self.dt       = float(get_or_declare('dt', 0.1))
        self.t_max    = float(get_or_declare('t_max', 600.0))

        self.map_file = map_file  # เก็บไว้ใช้ใน node

        # --- โหลดไฟล์แผนที่ ---
        path_file = os.path.join(
            get_package_share_directory('seer_robot_pkg'),
            'config',
            self.map_file
        )
        try:
            with open(path_file, 'r') as f:
                self.map_data = json.load(f)
            self.get_logger().info(f"Map data loaded from {self.map_file}")
        except FileNotFoundError:
            self.get_logger().error(f"Map file {self.map_file} not found at: {path_file}")
            rclpy.shutdown()
            return

        # --- แปลงแผนที่เป็น geometry: self.points, self.edges, (และถ้าทำใน buildmap: self.edge_bbox) ---
        _build_geometry_from_map(self, step=sampling_step)
        self.get_logger().info(f"Geometry ready: points={len(self.points)}, edges={len(self.edges)}")


    # ---- เสริม: normalize route tokens เช่น 'LM 22' -> 'LM22' ----
    def _normalize_route_tokens(self, tokens):
        out=[]; i=0
        valid = set(self.points.keys())
        while i < len(tokens):
            tok = tokens[i].strip()
            if i+1 < len(tokens) and tok in ("LM","AP","CP") and tokens[i+1].strip().isdigit():
                cand = tok + tokens[i+1].strip()
                if cand in valid:
                    out.append(cand); i+=2; continue
            merged = tok.replace(" ","")
            out.append(merged if merged in valid else tok)
            i += 1
        return out

    # ---- ต่อ polyline จากลิสต์ชื่อน็อด ----
    def build_route_polyline(self, nodes):
        seq = self._normalize_route_tokens(nodes)
        full=[]; missing=[]
        for a,b in zip(seq[:-1], seq[1:]):
            f = f"{a}-{b}"; r = f"{b}-{a}"
            if f in self.edges: seg = self.edges[f]
            elif r in self.edges: seg = list(reversed(self.edges[r]))
            else:
                missing.append((a,b)); continue
            full.extend(seg[1:] if full and seg and seg[0]==full[-1] else seg)
        if missing:
            hints=[]
            for a,b in missing:
                cand=[k for k in (f"{a}-{b}", f"{b}-{a}") if k in self.edges]
                hints.append(f"{a}<->{b} (candidates: {', '.join(cand) if cand else 'none'})")
            raise KeyError("Edge not found: " + "; ".join(hints))
        return full

    # ---- ตรวจชน: รถจอดที่ node vs รถวิ่งตาม route ----
    def check_collision_stationary_vs_route(self, stationary_node, route_nodes, v=1.0):
        if stationary_node not in self.points:
            raise KeyError(f"Unknown node: {stationary_node}")
        pose1 = self.points[stationary_node]                 # (x,y,yaw)
        W = self.get_parameter('robot_width').value
        L = self.get_parameter('robot_length').value
        buf = self.get_parameter('safety_buffer').value
        dt = self.get_parameter('dt').value
        t_max = self.get_parameter('t_max').value

        poly = self.build_route_polyline(route_nodes)
        cache = PolylineCache(poly)

        t = 0.0
        while t <= t_max:
            x,y,yaw = pose_along_polyline_cached(cache, v*t)
            if collide_OBB(pose1, (W,L), (x,y,yaw), (W,L), safety_buffer=buf):
                return {"collision": True, "t_hit": t, "pose2": (x,y,yaw)}
            t += dt
        return {"collision": False}

    # def _demo_once(self):
        # if self._demo_once_done: return
        # self._demo_once_done = True
        # try:
        #     res = self.check_collision_stationary_vs_route(
        #         "LM52",
        #         ["LM44","LM45","LM46","LM30","AP6","LM30","LM46","LM45","LM44","LM25","LM22","AP1","LM22"],
        #         v=1.0
        #     )
        #     self.get_logger().info(f"DEMO result: {res}")
        # except Exception as e:
        #     self.get_logger().error(str(e))

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

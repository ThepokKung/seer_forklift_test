#!/usr/bin/env python3
import os, json, math
from typing import Any, Dict, List, Tuple

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

        # ---------------- Params (ประกาศค่าเริ่มต้นแบบปลอดภัย) ----------------
        self._declare_param_safe('map_file', 'map.json')
        self._declare_param_safe('robot_parameter_file', 'forklift_parameter.json')
        self._declare_param_safe('step', 0.1)
        self._declare_param_safe('robot_width', 1.19)
        self._declare_param_safe('robot_length', 2.61)
        self._declare_param_safe('safety_buffer', 0.15)   # เผื่อขั้นต่ำ (ขั้นต่ำของระบบ)
        self._declare_param_safe('dt', 0.1)
        self._declare_param_safe('t_max', 600.0)

        # อ่านค่าพารามิเตอร์
        self.map_file  = self.get_parameter('map_file').get_parameter_value().string_value
        self.robot_parameter_file = self.get_parameter('robot_parameter_file').get_parameter_value().string_value
        self.step      = self.get_parameter('step').get_parameter_value().double_value
        self.robot_W   = self.get_parameter('robot_width').get_parameter_value().double_value
        self.robot_L   = self.get_parameter('robot_length').get_parameter_value().double_value
        self.buffer    = self.get_parameter('safety_buffer').get_parameter_value().double_value
        self.dt        = self.get_parameter('dt').get_parameter_value().double_value
        self.t_max     = self.get_parameter('t_max').get_parameter_value().double_value

        try:
            # ---------------- Load map ----------------
            map_path = self._resolve_pkg_path('seer_robot_pkg', 'configs', self.map_file)
            with open(map_path, 'r', encoding='utf-8') as f:
                self.map_data = json.load(f)
            self.get_logger().info(f"Map data loaded from {self.map_file}")

            # ---------------- Load robot parameter JSON ----------------
            robot_param_path = self._resolve_pkg_path('seer_robot_pkg', 'configs', self.robot_parameter_file)
            with open(robot_param_path, 'r', encoding='utf-8') as f:
                raw = json.load(f)
            self.fk = self._parse_forklift_params(raw)  # dict ค่าที่ TM ต้องใช้
            self.get_logger().info(f"Robot parameters loaded from {self.robot_parameter_file}")
        except FileNotFoundError as e:
            self.get_logger().error(f"File not found: {e.filename}")
            self.get_logger().error("Please check your configuration files.")
            self.get_logger().error("Node will not start due to missing configuration files.")
            self.destroy_node()
            rclpy.shutdown()
            return

        # ถ้ามี SimDt/SimTime ในไฟล์ ให้ override dt/t_max (optional)
        # if self.fk.get('sim_dt') is not None:
        #     self.dt = float(self.fk['sim_dt'])
        #     self.get_logger().info(f"Override dt from JSON: dt={self.dt}")
        # if self.fk.get('sim_time') is not None:
        #     self.t_max = float(self.fk['sim_time'])
        #     self.get_logger().info(f"Override t_max from JSON: t_max={self.t_max}")

        # ---------------- Build geometry -> self.points, self.edges, self.edge_bbox ----------------
        _build_geometry_from_map(self, step=self.step)
        self.get_logger().info(f"Geometry ready: points={len(self.points)}, edges={len(self.edges)}")  # type: ignore

        # ---------------- Demo timer (ลองชน: รถค้างที่ LM52 vs เส้นทาง) ----------------
        self._demo_done = False
        self.create_timer(1.0, self._demo_once)

    # ---------------- Utilities ----------------
    def _declare_param_safe(self, name: str, default: Any):
        try:
            self.declare_parameter(name, default)
        except ParameterAlreadyDeclaredException:
            # ถูกประกาศแล้ว (จาก launch/yaml) ก็อ่านค่าที่มีอยู่
            pass

    def _resolve_pkg_path(self, pkg: str, subdir: str, filename: str) -> str:
        """พยายามอ่านจาก share ของแพ็กเกจก่อน ถ้าไม่เจอ fallback เป็น path ข้างสคริปต์"""
        try:
            path = os.path.join(get_package_share_directory(pkg), subdir, filename)
            if os.path.exists(path):
                return path
        except Exception:
            pass
        # fallback: <pkg_dir_guess>/<subdir>/<filename>
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_dir = os.path.dirname(script_dir)
        path = os.path.join(pkg_dir, subdir, filename)
        if not os.path.exists(path):
            raise FileNotFoundError(f"File not found: {path}")
        return path

    # ---------------- Forklift parameter parsing ----------------
    def _parse_forklift_params(self, raw: Dict[str, Any]) -> Dict[str, Any]:
        """
        ดึงเฉพาะคีย์ที่ TM ต้องใช้ (ทนทานต่อโครง JSON ที่ต่างกันเล็กน้อย):
          - max_speed (empty) / max_speed_loaded
          - obs_expansion / obs_expansion_loaded
          - obs_dec_dist / obs_dec_dist_loaded
          - obs_stop_dist / obs_stop_dist_loaded
          - stop_acc / stop_acc_loaded (ถ้ามี)
          - sim_dt / sim_time (ถ้ามี)
        รองรับทั้งกรณีคีย์อยู่ใต้ 'MoveFactory' หรืออยู่ระดับ root
        """
        # ถ้ามี MoveFactory ให้เจาะเข้าไป
        src = raw.get('MoveFactory', raw)

        def _val(*names, default=None):
            """
            หา value แบบยืดหยุ่น: รองรับรูปแบบ:
              {"MaxSpeed": 1.0} หรือ {"MaxSpeed": {"value": 1.0}}
            และรองรับ key case/ชื่อใกล้เคียง
            """
            for nm in names:
                if nm in src:
                    v = src[nm]
                    if isinstance(v, dict) and 'value' in v:
                        return v['value']
                    return v
            return default

        out = {
            # speed
            'max_speed'          : _val('MaxSpeed', 'max_speed', default=None),
            'max_speed_loaded'   : _val('Load_MaxSpeed', 'Load_max_speed', 'loaded_max_speed', default=None),

            # safety expansion (buffer จากไฟล์)
            'obs_expansion'          : _val('ObsExpansion', 'obs_expansion', default=None),
            'obs_expansion_loaded'   : _val('Load_ObsExpansion', 'Load_obs_expansion', 'loaded_obs_expansion', default=None),

            # decision/stop distances (อาจใช้ในนโยบาย traffic เพิ่มภายหลัง)
            'obs_dec_dist'           : _val('ObsDecDist', 'obs_dec_dist', default=None),
            'obs_dec_dist_loaded'    : _val('Load_ObsDecDist', 'Load_obs_dec_dist', default=None),
            'obs_stop_dist'          : _val('ObsStopDist', 'obs_stop_dist', default=None),
            'obs_stop_dist_loaded'   : _val('Load_ObsStopDist', 'Load_obs_stop_dist', default=None),

            # accel (เบรกฉุกเฉิน) – ไว้คำนวณระยะหยุด worst-case ถ้าต้องใช้
            'stop_acc'               : _val('StopAcc', 'stop_acc', default=None),
            'stop_acc_loaded'        : _val('Load_StopAcc', 'Load_stop_acc', default=None),

            # simulation time base
            'sim_dt'                 : _val('SimDt', 'sim_dt', default=None),
            'sim_time'               : _val('SimTime', 'sim_time', default=None),
        }

        # แปลงเป็น float ให้ได้มากที่สุด
        for k, v in list(out.items()):
            if v is None: 
                continue
            try:
                out[k] = float(v) # type: ignore
            except Exception:
                # ถ้าเป็น string ที่ห่อ value ไว้ แก้แบบเบาๆ
                try:
                    out[k] = float(str(v).strip())
                except Exception:
                    out[k] = None
        return out

    def _normalize_route_tokens(self, tokens: List[str]) -> List[str]:
        out=[]; i=0
        valid = set(self.points.keys())  # type: ignore
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

    def build_route_polyline(self, nodes: List[str]):
        seq = self._normalize_route_tokens(nodes)
        full=[]; missing=[]
        for a,b in zip(seq[:-1], seq[1:]):
            f=f"{a}-{b}"; r=f"{b}-{a}"
            if f in self.edges: seg = self.edges[f]  # type: ignore
            elif r in self.edges: seg = list(reversed(self.edges[r]))  # type: ignore
            else:
                missing.append((a,b)); continue
            full.extend(seg[1:] if full and seg and full[-1]==seg[0] else seg)
        if missing:
            hints=[]
            for a,b in missing:
                cand=[k for k in (f"{a}-{b}", f"{b}-{a}") if k in self.edges]  # type: ignore
                hints.append(f"{a}<->{b} (candidates: {', '.join(cand) if cand else 'none'})")
            raise KeyError("Edge not found: " + "; ".join(hints))
        return full

    # ---------------- เลือกความเร็ว/บัฟเฟอร์จากไฟล์ (พร้อม fallback) ----------------
    def _pick_speed(self, loaded: bool, fallback_v: float = 1.0) -> float:
        v_loaded  = self.fk.get('max_speed_loaded')
        v_empty   = self.fk.get('max_speed')
        if loaded and v_loaded is not None:
            return float(v_loaded)
        if (not loaded) and v_empty is not None:
            return float(v_empty)
        return float(fallback_v)

    def _pick_buffer(self, loaded: bool, base_min: float) -> float:
        """คืน safety_buffer ที่จะส่งเข้า collide_OBB (เมตร)"""
        b_loaded = self.fk.get('obs_expansion_loaded')
        b_empty  = self.fk.get('obs_expansion')
        from_json = None
        if loaded and b_loaded is not None:
            from_json = float(b_loaded)
        elif (not loaded) and b_empty is not None:
            from_json = float(b_empty)
        # ใช้ค่าที่ใหญ่กว่า ระหว่าง base_min (param ของ node) กับค่าในไฟล์
        if from_json is None:
            return float(base_min)
        return float(max(base_min, from_json))

    # ---------------- API หลัก: ตรวจชน รถค้างที่ node vs รถวิ่งตาม route ----------------
    def check_collision_stationary_vs_route(
        self,
        stationary_node: str,
        route_nodes: List[str],
        v: float | None = None,
        loaded: bool = False,
    ) -> Dict[str, Any]:
        """
        ถ้าไม่ส่ง v มา: จะเลือกจากไฟล์ forklift_parameter.json (Loaded/Empty)
        safety_buffer: เลือกจากไฟล์ (ObsExpansion / Load_ObsExpansion) แต่จะไม่ต่ำกว่าพารามิเตอร์ของ node
        """
        if stationary_node not in self.points:  # type: ignore
            raise KeyError(f"Unknown node: {stationary_node}")

        x1, y1, yaw1 = self.points[stationary_node]  # type: ignore
        W, L = self.robot_W, self.robot_L
        dt, tmax = self.dt, self.t_max

        # เลือก v / buffer จากไฟล์ (พร้อม fallback)
        v_run = self._pick_speed(loaded, fallback_v = (v if (v is not None) else 1.0))
        buf   = self._pick_buffer(loaded, base_min=self.buffer)

        # ต่อ polyline ของ route
        poly = self.build_route_polyline(route_nodes)

        # ไล่เวลาจำลอง
        t=0.0
        while t <= tmax:
            x2, y2, yaw2 = pose_along_polyline(poly, v_run * t)
            if collide_OBB((x1,y1,yaw1), (W,L), (x2,y2,yaw2), (W,L), safety_buffer=buf):
                return {
                    "collision": True,
                    "t_hit": t,
                    "pose2": (x2,y2,yaw2),
                    "v_used": v_run,
                    "buffer_used": buf,
                    "loaded": loaded,
                }
            t += dt
        return {"collision": False, "v_used": v_run, "buffer_used": buf, "loaded": loaded}

    # ---------------- Demo ----------------
    def _demo_once(self):
        if self._demo_done: 
            return
        self._demo_done = True
        try:
            route = ['LM44','LM45','LM46','LM30','AP6','LM30','LM46','LM45','LM44','LM25','LM22','AP1','LM22']
            res = self.check_collision_stationary_vs_route('LM52', route, v=None, loaded=False)
            if res["collision"]:
                self.get_logger().warn(f"[DEMO] COLLISION t={res['t_hit']:.2f}s  v={res['v_used']:.2f}  buf={res['buffer_used']:.2f}")
            else:
                self.get_logger().info(f"[DEMO] SAFE  v={res['v_used']:.2f}  buf={res['buffer_used']:.2f}")
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

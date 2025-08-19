from .map_utils import build_polylines_from_map, build_route_polyline, get_node_pose
from .geometry_utils import pose_along_polyline
from .collision_utils import collide_OBB

def simulate(map_path, stationary_node, route_nodes, v2=1.0, W=1.19, L=2.61,
             buffer=0.15, dt=0.1, t_max=600.0, sample_step=0.1):
    polys, data = build_polylines_from_map(map_path, step=sample_step)
    poly2 = build_route_polyline(polys, route_nodes, data=data)
    pose1 = get_node_pose(data, stationary_node)
    t = 0.0
    while t <= t_max:
        s2 = v2 * t
        x2,y2,yaw2 = pose_along_polyline(poly2, s2)
        if collide_OBB(pose1, (W,L), (x2,y2,yaw2), (W,L), safety_buffer=buffer):
            return {
                "collision": True,
                "t_hit": t,
                "pose_robot2": (x2, y2, yaw2),
                "stationary_pose": pose1,
            }
        t += dt
    return {"collision": False, "t_hit": None}

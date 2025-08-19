# seer_robot_pkg/seer_robot_pkg/collision_geom.py
import math, bisect
from typing import List, Tuple

Point = Tuple[float, float]

# --- Bezier and sampling utilities ---
def _bezier_point(p0, p1, p2, p3, t: float) -> Point:
    mt = 1.0 - t
    x = (mt**3)*p0[0] + 3*(mt**2)*t*p1[0] + 3*mt*(t**2)*p2[0] + (t**3)*p3[0]
    y = (mt**3)*p0[1] + 3*(mt**2)*t*p1[1] + 3*mt*(t**2)*p2[1] + (t**3)*p3[1]
    return (x, y)

def _seg_len(a:Point, b:Point) -> float:
    return math.hypot(b[0]-a[0], b[1]-a[1])

def sample_bezier_uniform(p0, p1, p2, p3, step:float=0.1) -> List[Point]:
    """Oversample cubic Bezier then keep points ~step apart (meters)."""
    N = 160
    coarse = [_bezier_point(p0,p1,p2,p3, i/(N-1)) for i in range(N)]
    out = [coarse[0]]
    acc = 0.0
    for i in range(1, len(coarse)):
        d = _seg_len(coarse[i-1], coarse[i])
        acc += d
        if acc >= step:
            out.append(coarse[i])
            acc = 0.0
    if out[-1] != coarse[-1]:
        out.append(coarse[-1])
    return out

def sample_straight(p0:Point, p3:Point, step:float=0.1) -> List[Point]:
    total = _seg_len(p0, p3)
    if total == 0: return [p0]
    n = max(2, int(round(total/step))+1)
    return [(p0[0]+(p3[0]-p0[0])*(i/(n-1)), p0[1]+(p3[1]-p0[1])*(i/(n-1))) for i in range(n)]

# --- Polyline pose helpers (cached) ---
def cumulative_lengths(pts: List[Point]) -> List[float]:
    acc=0.0; out=[0.0]
    for i in range(1, len(pts)):
        acc += _seg_len(pts[i-1], pts[i])
        out.append(acc)
    return out

class PolylineCache:
    def __init__(self, pts: List[Point]):
        self.pts = pts
        self.cum = cumulative_lengths(pts)
        self.L   = self.cum[-1] if self.cum else 0.0

def pose_along_polyline(pts: List[Point], s: float):
    """Return (x,y,yaw) along pts at arc-length s (m)."""
    if not pts: return (0.0,0.0,0.0)
    cum = cumulative_lengths(pts); L = cum[-1]
    if s <= 0:
        a=pts[0]; b=pts[min(1,len(pts)-1)]
        yaw = math.atan2(b[1]-a[1], b[0]-a[0]) if len(pts)>1 else 0.0
        return a[0],a[1],yaw
    if s >= L:
        a=pts[-2] if len(pts)>1 else pts[0]; b=pts[-1]
        yaw = math.atan2(b[1]-a[1], b[0]-a[0]) if len(pts)>1 else 0.0
        return b[0],b[1],yaw
    i=max(1, bisect.bisect_left(cum, s))
    a,b = pts[i-1], pts[i]
    ds = s - cum[i-1]; seg=_seg_len(a,b)
    t=0.0 if seg==0 else ds/seg
    x = a[0] + (b[0]-a[0])*t
    y = a[1] + (b[1]-a[1])*t
    yaw = math.atan2(b[1]-a[1], b[0]-a[0])
    return x,y,yaw

def pose_along_polyline_cached(cache: PolylineCache, s: float):
    pts, cum, L = cache.pts, cache.cum, cache.L
    if not pts: return (0.0,0.0,0.0)
    if s <= 0:
        a=pts[0]; b=pts[min(1,len(pts)-1)]
        yaw = math.atan2(b[1]-a[1], b[0]-a[0]) if len(pts)>1 else 0.0
        return a[0],a[1],yaw
    if s >= L:
        a=pts[-2] if len(pts)>1 else pts[0]; b=pts[-1]
        yaw = math.atan2(b[1]-a[1], b[0]-a[0]) if len(pts)>1 else 0.0
        return b[0],b[1],yaw
    i=max(1, bisect.bisect_left(cum, s))
    a,b = pts[i-1], pts[i]
    ds = s - cum[i-1]; seg=_seg_len(a,b)
    t=0.0 if seg==0 else ds/seg
    x = a[0] + (b[0]-a[0])*t
    y = a[1] + (b[1]-a[1])*t
    yaw = math.atan2(b[1]-a[1], b[0]-a[0])
    return x,y,yaw

# --- OBB + SAT ---
def obb_corners(x, y, yaw, W, L):
    hw, hl = W/2.0, L/2.0
    local = [(-hl,-hw), (-hl,hw), (hl,hw), (hl,-hw)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + dx*c - dy*s, y + dx*s + dy*c) for dx,dy in local]

def _proj_interval(corners, axis):
    dots = [cx*axis[0] + cy*axis[1] for cx,cy in corners]
    return min(dots), max(dots)

def _intervals_overlap(a, b): return not (a[1] < b[0] or b[1] < a[0])

def _norm(vx, vy):
    n = math.hypot(vx, vy)
    return (vx/n, vy/n) if n else (0.0, 0.0)

def obb_intersect(c1, c2):
    axes = []
    for corners in (c1, c2):
        for i in (0,1):
            x1,y1 = corners[i]; x2,y2 = corners[(i+1)%4]
            ex,ey = x2-x1, y2-y1
            nx,ny = -ey, ex
            axes.append(_norm(nx,ny))
    for ax in axes:
        a1=_proj_interval(c1, ax); a2=_proj_interval(c2, ax)
        if not _intervals_overlap(a1,a2):
            return False
    return True

def _aabb_of_obb(corners):
    xs=[c[0] for c in corners]; ys=[c[1] for c in corners]
    return (min(xs), min(ys), max(xs), max(ys))

def _aabb_overlap(a,b):
    return not (a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1])

def collide_OBB(poseA, sizeA, poseB, sizeB, safety_buffer=0.0):
    (xA,yA,yawA),(W1,L1) = poseA,sizeA
    (xB,yB,yawB),(W2,L2) = poseB,sizeB
    # circle quick reject
    r1=0.5*math.hypot(W1,L1); r2=0.5*math.hypot(W2,L2)
    if math.hypot(xB-xA, yB-yA) > (r1+r2+safety_buffer):
        return False
    # AABB quick reject
    c1=obb_corners(xA,yA,yawA,W1,L1); c2=obb_corners(xB,yB,yawB,W2,L2)
    a1=_aabb_of_obb(c1); a2=_aabb_of_obb(c2)
    a1=(a1[0]-safety_buffer,a1[1]-safety_buffer,a1[2]+safety_buffer,a1[3]+safety_buffer)
    a2=(a2[0]-safety_buffer,a2[1]-safety_buffer,a2[2]+safety_buffer,a2[3]+safety_buffer)
    if not _aabb_overlap(a1,a2):
        return False
    return obb_intersect(c1,c2)

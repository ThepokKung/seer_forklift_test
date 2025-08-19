import math,bisect

def bezier_point(p0, p1, p2, p3, t):
    """Return point on cubic Bezier curve at parameter t (0 <= t <= 1)"""
    x = (
        (1-t)**3 * p0[0] +
        3*(1-t)**2 * t * p1[0] +
        3*(1-t) * t**2 * p2[0] +
        t**3 * p3[0]
    )
    y = (
        (1-t)**3 * p0[1] +
        3*(1-t)**2 * t * p1[1] +
        3*(1-t) * t**2 * p2[1] +
        t**3 * p3[1]
    )
    return (x, y)

def seg_len(a, b):
    return math.hypot(b[0]-a[0], b[1]-a[1])

def sample_bezier_uniform(p0, p1, p2, p3, step=0.1):
    N = 160
    coarse = [bezier_point(p0,p1,p2,p3, i/(N-1)) for i in range(N)]
    out = [coarse[0]]; acc = 0.0
    for i in range(1, len(coarse)):
        d = seg_len(coarse[i-1], coarse[i])
        acc += d
        if acc >= step:
            out.append(coarse[i]); acc = 0.0
    if out[-1] != coarse[-1]:
        out.append(coarse[-1])
    return out

def sample_straight(p0, p3, step=0.1):
    total = seg_len(p0, p3)
    if total == 0: return [p0]
    n = max(2, int(round(total/step))+1)
    return [(p0[0] + (p3[0]-p0[0])*(i/(n-1)), p0[1] + (p3[1]-p0[1])*(i/(n-1))) for i in range(n)]

def cumulative_lengths(pts):
    acc = 0.0; out = [0.0]
    for i in range(1, len(pts)):
        acc += seg_len(pts[i-1], pts[i]); out.append(acc)
    return out

def pose_along_polyline(pts, s):
    if not pts: return (0.0,0.0,0.0)
    cum = cumulative_lengths(pts); Lp = cum[-1]
    if s <= 0:
        a, b = pts[0], pts[min(1, len(pts)-1)]
        yaw = math.atan2(b[1]-a[1], b[0]-a[0]) if len(pts)>1 else 0.0
        return a[0], a[1], yaw
    if s >= Lp:
        a = pts[-2] if len(pts)>1 else pts[0]; b = pts[-1]
        yaw = math.atan2(b[1]-a[1], b[0]-a[0]) if len(pts)>1 else 0.0
        return b[0], b[1], yaw
    i = max(1, bisect.bisect_left(cum, s))
    a, b = pts[i-1], pts[i]
    ds = s - cum[i-1]; seg = seg_len(a,b)
    t = 0.0 if seg==0 else ds/seg
    x = a[0] + (b[0]-a[0])*t; y = a[1] + (b[1]-a[1])*t
    yaw = math.atan2(b[1]-a[1], b[0]-a[0])
    return x, y, yaw

import math

def obb_corners(x, y, yaw, W, L):
    hw, hl = W/2.0, L/2.0
    local = [(-hl,-hw), (-hl,hw), (hl,hw), (hl,-hw)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + dx*c - dy*s, y + dx*s + dy*c) for dx,dy in local]

def _proj_interval(corners, axis):
    dots = [cx*axis[0] + cy*axis[1] for cx,cy in corners]
    return min(dots), max(dots)

def _intervals_overlap(a, b):
    return not (a[1] < b[0] or b[1] < a[0])

def _norm(vx, vy):
    n = math.hypot(vx, vy)
    return (vx/n, vy/n) if n else (0.0, 0.0)

def obb_intersect(c1, c2):
    axes = []
    for corners in (c1, c2):
        for i in range(4):
            x1,y1 = corners[i]; x2,y2 = corners[(i+1)%4]
            ex, ey = x2-x1, y2-y1
            nx, ny = -ey, ex
            axes.append(_norm(nx, ny))
    for ax in axes:
        if not _intervals_overlap(_proj_interval(c1, ax), _proj_interval(c2, ax)):
            return False
    return True

def collide_OBB(poseA, sizeA, poseB, sizeB, safety_buffer=0.0):
    (xA,yA,yawA), (W1,L1) = poseA, sizeA
    (xB,yB,yawB), (W2,L2) = poseB, sizeB
    r1 = 0.5*math.hypot(W1, L1); r2 = 0.5*math.hypot(W2, L2)
    if math.hypot(xB-xA, yB-yA) > (r1 + r2 + safety_buffer):
        return False
    c1 = obb_corners(xA, yA, yawA, W1, L1)
    c2 = obb_corners(xB, yB, yawB, W2, L2)
    return obb_intersect(c1, c2)

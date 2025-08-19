import json
from pathlib import Path

def load_map(path):
    return json.loads(Path(path).read_text())

def build_polylines_from_map(path, step=0.1):
    from .geometry_utils import sample_straight, sample_bezier_uniform
    data = load_map(path)
    polylines = {}
    for c in data.get("advancedCurveList", []):
        cls = c.get("className")
        name = c.get("instanceName")
        sp = c["startPos"]["pos"]; ep = c["endPos"]["pos"]
        p0 = (float(sp["x"]), float(sp["y"]))
        p3 = (float(ep["x"]), float(ep["y"]))
        if cls == "StraightPath":
            pts = sample_straight(p0, p3, step=step)
        elif cls == "BezierPath":
            cp1 = c.get("controlPos1", {}); cp2 = c.get("controlPos2", {})
            p1 = (float(cp1["x"]), float(cp1["y"]))
            p2 = (float(cp2["x"]), float(cp2["y"]))
            pts = sample_bezier_uniform(p0, p1, p2, p3, step=step)
        else:
            pts = sample_straight(p0, p3, step=step)
        polylines[name] = pts
    return polylines, data

def get_node_pose(data, name):
    for p in data.get("advancedPointList", []):
        if p.get("instanceName") == name:
            x = float(p["pos"]["x"]); y = float(p["pos"]["y"])
            yaw = float(p.get("dir", 0.0))
            return x, y, yaw
    raise KeyError(f"Node not found: {name}")

def _normalize_route_tokens(tokens, valid_nodes):
    out = []
    i = 0
    while i < len(tokens):
        tok = tokens[i].strip()
        if i+1 < len(tokens) and tok in ("LM","AP","CP") and tokens[i+1].strip().isdigit():
            cand = tok + tokens[i+1].strip()
            if cand in valid_nodes:
                out.append(cand)
                i += 2
                continue
        merged = tok.replace(" ", "")
        if merged in valid_nodes:
            out.append(merged)
            i += 1
            continue
        out.append(tok)
        i += 1
    return out

def build_route_polyline(polys, sequence, data=None):
    valid_nodes = set(p.get("instanceName","") for p in (data.get("advancedPointList", []) if data else []))
    seq = _normalize_route_tokens(sequence, valid_nodes) if valid_nodes else list(sequence)
    full = []
    missing = []
    for a, b in zip(seq[:-1], seq[1:]):
        name_fwd = f"{a}-{b}"; name_rev = f"{b}-{a}"
        if name_fwd in polys:
            seg = polys[name_fwd]
        elif name_rev in polys:
            seg = list(reversed(polys[name_rev]))
        else:
            missing.append((a,b))
            continue
        if full and seg and full[-1] == seg[0]:
            full.extend(seg[1:])
        else:
            full.extend(seg)
    if missing:
        known = set(polys.keys())
        hints = []
        for a,b in missing:
            cand = [k for k in (f"{a}-{b}", f"{b}-{a}") if k in known]
            hints.append(f"{a}<->{b} (candidates: {', '.join(cand) if cand else 'none'})")
        raise KeyError("Edge not found: " + "; ".join(hints))
    return full

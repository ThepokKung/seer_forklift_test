from seer_robot_pkg.collision_geom import sample_bezier_uniform, sample_straight

def _build_geometry_from_map(self, step=0.1):
    """Populate self.points, self.edges, self.edge_bbox from self.map_data (Seer map JSON)."""
    self.points = {}     # node -> (x,y,yaw)
    self.edges  = {}     # "A-B" -> [(x,y), ...]
    self.edge_bbox = {}  # "A-B" -> (minx,miny,maxx,maxy)

    for p in self.map_data.get("advancedPointList", []):
        name = p.get("instanceName", "")
        x = float(p["pos"]["x"]); y = float(p["pos"]["y"]); yaw = float(p.get("dir", 0.0))
        self.points[name] = (x, y, yaw)

    for c in self.map_data.get("advancedCurveList", []):
        cls = c.get("className"); name = c.get("instanceName")
        sp = c["startPos"]["pos"]; ep = c["endPos"]["pos"]
        p0 = (float(sp["x"]), float(sp["y"])); p3 = (float(ep["x"]), float(ep["y"]))
        if cls == "StraightPath":
            pts = sample_straight(p0, p3, step)
        elif cls == "BezierPath":
            cp1 = c.get("controlPos1", {}); cp2 = c.get("controlPos2", {})
            p1 = (float(cp1["x"]), float(cp1["y"])); p2 = (float(cp2["x"]), float(cp2["y"]))
            pts = sample_bezier_uniform(p0, p1, p2, p3, step)
        else:
            pts = sample_straight(p0, p3, step)
        self.edges[name] = pts
        xs=[x for x,_ in pts]; ys=[y for _,y in pts]
        self.edge_bbox[name] = (min(xs), min(ys), max(xs), max(ys))

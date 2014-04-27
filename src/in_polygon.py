from shapely.geometry import MultiPoint, Polygon

def in_polygon(points, polygon_corners):
    polygon = Polygon(polygon_corners)
    shapely_multipoint = MultiPoint(points)
    if polygon.intersects(shapely_multipoint):
        return True 
    return False

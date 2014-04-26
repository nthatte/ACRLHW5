from shapely.geometry import Point, Polygon

def in_polygon(points, polygon_corners):
    polygon = Polygon(polygon_corners)
    for point in points:
        shapely_point = Point(point)
        if polygon.intersects(shapely_point):
            return True 
    return False

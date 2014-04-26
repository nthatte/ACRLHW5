import shapely

def in_polygon(points, polygon_corners):
    polygon = shapely.geometry.Polygon(polygon_corners)
    for point in points:
        shapely_point = shapely.geometry.Point(point)
        if polygon.intersects(point):
            return true 
    return false

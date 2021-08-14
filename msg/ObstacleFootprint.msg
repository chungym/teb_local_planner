# Message that contains obstacle footprint 

# Obstacle Type
uint8 PointObstacle=1 
uint8 CircularObstacle=2 
uint8 LineObstacle=3 
uint8 PillObstacle=4 
uint8 PolygonObstacle=5 

int64 type

bool holonomic

# Parameter for LineObstacle, PillObstacle
geometry_msgs/Point point1

# Parameter for LineObstacle, PillObstacle
geometry_msgs/Point point2

# Radius of point1
# Parameter for CircularObstacle, PillObstacle
float64 radius

# Obstacle footprint (polygon descriptions)
geometry_msgs/Polygon polygon
uint32 RLEG = 0
uint32 LLEG = 1


Header header
geometry_msgs/PolygonStamped[] polygons
uint32[] labels
float32[] likelihood
uint32 l_r # 0: right. 1: left. 現在支持脚

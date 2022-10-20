# この型は safe_footstep_planner/msg/SteppableRegion.msg と同一である必要がある

Header header
geometry_msgs/PolygonStamped[] polygons
uint32[] labels
float32[] likelihood
uint32 l_r # 0: right. 1: left. 現在支持脚

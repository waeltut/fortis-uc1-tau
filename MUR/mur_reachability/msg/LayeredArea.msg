# Grid in header.frame_id; origin is the lower-left corner, axes match that frame.
# One layer per base yaw. x-fastest: x + width*(y + height*heading_index).
std_msgs/Header header
string arm
string cache_id
bool valid
float64 resolution
geometry_msgs/Point origin
uint32 width
uint32 height
float64 base_z
float64[] headings
# 0 unknown/not returned (NOT proven unreachable)
# 1 approximate cached candidate
# 2 exact endpoint IK + self-collision verified at cell centre and this heading
uint8[] cells
geometry_msgs/PoseStamped target
bool lookup_truncated
bool verification_budget_exhausted
string conditions

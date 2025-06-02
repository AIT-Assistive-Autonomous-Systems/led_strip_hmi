# led_strip_hmi_msgs/msg/LEDStripProjectionInfo.msg

std_msgs/Header header
uint32          detection_id

# normalized [0..1] position along the *entire* virtual strip
float32         peak
float32         start
float32         stop

# actual sensor range to that detection in meters
float32         distance

# Optional lifetime in seconds. If has_lifetime==false, ignore lifetime.
bool    has_lifetime
float32 lifetime

# embed the new effect descriptor:
LEDStripEffect  effect
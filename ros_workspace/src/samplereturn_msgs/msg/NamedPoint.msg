# This represents a Point with a reference frame, timestamp, and name

int8 NONE=0
int8 PRE_CACHED=1
int8 PINK_TENNIS_BALL=2
int8 RED_PUCK=3
int8 WOODEN_CUBE=4
int8 BLUE_ROCK=5
int8 METAL_1=6
int8 METAL_2=7
int8 ORANGE_PIPE=8
int8 COLORED_BALL=9
int8 METAL_3=10

Header header
geometry_msgs/Point point
# Replace color name with color model
samplereturn_msgs/ColorModel model

# This is a bad workaround to avoid touching well-tested executive code
string sensor_frame

int16 filter_id
float32 grip_angle

# To be phased out
int8 sample_id
string name

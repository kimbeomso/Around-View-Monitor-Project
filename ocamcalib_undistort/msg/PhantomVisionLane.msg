uint32 id
bool valid
bool measured

uint8 LANE_MARK_DASHED = 0
uint8 LANE_MARK_SOLID = 1
uint8 LANE_MARK_UNDECIDED = 2
uint8 LANE_MARK_ROAD_EDGE = 3
uint8 LANE_MARK_DOUBLE_LANE_MARK = 4
uint8 LANE_MARK_BOTTS_DOTTS = 5
uint8 LANE_MARK_INVALID = 6
uint8 mark_type

uint8 LANE_QUALITY_UNAVAILABLE = 0
uint8 LANE_QUALITY_LOW = 1
uint8 LANE_QUALITY_MEDIUM = 2
uint8 LANE_QUALITY_HIGH = 3
uint8 quality

int8 LANE_LOCATION_UNKNOWN = 0
int8 LANE_LOCATION_LEFT = 1
int8 LANE_LOCATION_RIGHT = -1
int8 LANE_LOCATION_NEXT_LEFT = 2
int8 LANE_LOCATION_NEXT_RIGHT = -2
int8 LANE_LOCATION_FAR_LEFT = 3
int8 LANE_LOCATION_FAR_RIGHT = -3
int8 location

# World coordinates
float32 c0
float32 c1
float32 c2
float32 c3

float32 width_m
float32 view_range_start_m
float32 view_range_end_m

float32 age_secs
float32 pitch_rads

# Image coordinates
string camera_name
uint8 camera_id
float32[] x_px
float32[] y_px

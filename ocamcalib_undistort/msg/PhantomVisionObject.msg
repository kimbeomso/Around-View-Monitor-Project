uint32 id
bool valid
bool measured

# Comprehensive list of all object classes from image annotation tasks
uint8 CLASSIFICATION_UNKNOWN = 0
uint8 CLASSIFICATION_CAR = 1
uint8 CLASSIFICATION_TRUCK = 2
uint8 CLASSIFICATION_MOTORCYCLE = 3
uint8 CLASSIFICATION_PEDESTRIAN = 4
uint8 CLASSIFICATION_OTHER = 5
uint8 CLASSIFICATION_WHEEL = 6
uint8 CLASSIFICATION_CART = 7
uint8 CLASSIFICATION_SEDAN = 10
uint8 CLASSIFICATION_SUV = 11
uint8 CLASSIFICATION_VAN = 12
uint8 CLASSIFICATION_TRAILER = 13
uint8 CLASSIFICATION_BUS = 14
uint8 CLASSIFICATION_GLARE = 15
uint8 CLASSIFICATION_GENERAL = 16
uint8 CLASSIFICATION_STOPPER = 8
uint8 CLASSIFICATION_POLE = 9
uint8 CLASSIFICATION_BICYCLE = 17
uint8 CLASSIFICATION_BICYCLIST = 18
uint8 CLASSIFICATION_MOTORCYCLIST = 19
uint8 classification

uint8 VIEW_FACE_UNKNOWN = 0
uint8 VIEW_FACE_WHOLE = 1
uint8 VIEW_FACE_FRONT = 2
uint8 VIEW_FACE_REAR = 3
uint8 VIEW_FACE_SIDE = 4
uint8 VIEW_FACE_SIDE_LEFT = 5
uint8 VIEW_FACE_SIDE_RIGHT = 6
uint8 view_face

int8 LANE_ASSIGNMENT_UNKNOWN = 127
int8 LANE_ASSIGNMENT_CENTER = 0
int8 LANE_ASSIGNMENT_LEFT = 1
int8 LANE_ASSIGNMENT_RIGHT = -1
int8 LANE_ASSIGNMENT_NEXT_LEFT = 2
int8 LANE_ASSIGNMENT_NEXT_RIGHT = -2
int8 LANE_ASSIGNMENT_FAR_LEFT = 3
int8 LANE_ASSIGNMENT_FAR_RIGHT = -3
int8 lane_assignment

# Top-N classification and their confidences in [0,1]
uint8[] classification_candidates
float32[] classification_confidences

# Camera-centered world coordinates
float32 rel_velocity_x_mps
float32 rel_velocity_y_mps
float32 position_y_m
float32 position_x_m

float32 range_velocity_mps
float32 range_m
float32 angle_rads

float32 width_m
float32 length_m
float32 orientation_rads

bool total_left_bounding_angle_valid
bool total_right_bounding_angle_valid
float32 total_left_bounding_angle_rads
float32 total_right_bounding_angle_rads

bool face_left_bounding_angle_valid
bool face_right_bounding_angle_valid
float32 face_left_bounding_angle_rads
float32 face_right_bounding_angle_rads

float32 age_secs
float32 time_to_collision_secs
bool cipv
bool oncoming_traffic
bool cross_traffic

# Image coordinates
string camera_name
uint8 camera_id
float32 x_px
float32 y_px
float32 width_px
float32 height_px
std_msgs/Header header

string camera_name # Unique identifier of this camera. The name should match the camera's namespace.
int32[] resolution # [width, height]
string frame_id # Optional. Overrides the frame_id of image messages

string camera_model # Projection model
float64[] intrinsics # Vector with intrinsics parameters for the chosen projection model

string distortion_model # Distortion model
float64[] distortion_coeffs # Distortion model coefficients

sensor_msgs/Image mask # Optional. Binary image mask. White pixels are valid, black pixels are invalid.

Header header

string camera_name # unique identifier of camera. should match camera ns
int32[] resolution # [width, height]
string frame_id # Optional. Overrides image frame_id

string camera_model # projection model
float64[] intrinsics # vector with intrinsics parameter. parameters have to be applicable for camera_model

string distortion_model # model for distortion
float64[] distortion_coeffs # distortion model coefficients

sensor_msgs/Image mask # Optional. Binary image mask. White pixels are valid, black pixels are invalid.

# The msg type for sending cost map information
# length is the dimension on x-direction
# width is the dimension on y-direction
# time is the time stamp
# data is the map dictionary transformed to 1-D vector

int16     length
int16     width
int16     time
float64[] data
# The raw 16 bit data in each pixel
std_msgs/Header header
uint32 height   #height of thermal image
uint32 width    #width of thermal image
uint16[] data   #actual raw data, size is height*width

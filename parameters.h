


//important parameters
float  staticness_th ;// Staticness score threshold (degree of the object being static)
double objectness_th;//  Objectness score threshold (probability that the rectangle contain an object)
int aotime ; //aotime*framemod2= number of frames the objects must be  static// if set too low, maybe cause false detections !! 
int aotime2 ;// Half or more of aotime
double alpha ; // background scene learning rate 
double fore_th; // Threshold moving edges segmentation


//Less important parameters
int frameinit ; // Frames needed for learning the background
bool low_light; // if night scene

int framemod ;
int framemod2 ;
uint minsize; // Static object minimum size
float resize_scale ;// Image resize scale

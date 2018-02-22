


//important parameters
float  staticness_th = 0.00001;// Staticness score threshold (degree of the object being static)
double objectness_th = 0.00001;//  Objectness score threshold (probability that the rectangle contain an object)
int aotime = 30; //aotime*framemod2= number of frames the objects must be  static// if set too low, maybe cause false detections !! 
double alpha = 0.0002; // background scene learning rate 
double fore_th = 50; // Threshold moving edges segmentation

//Less important parameters
int frameinit = 300; // Frames needed for learning the background
bool low_light = false; // if night scene

int framemod = 1;
int framemod2 = 5;
uint minsize=30; // Static object minimum size
float resize_scale=0.5 ;// Image resize scale
int aotime2 = (aotime*2)/3;// Half or more of aotime

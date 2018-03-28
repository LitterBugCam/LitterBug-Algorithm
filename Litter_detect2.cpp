
#include "edge_grouping.h"
#include "scoring.h"
#include "parameters.h"
#include <omp.h>

objects abandoned_objects;

Mat image, gray, F;
Mat grad_x, D_Sx, B_Sx, F_Sx;
Mat grad_y, D_Sy, B_Sy, F_Sy;
Mat result, threshed1, accumulation;
int vis;
float meanfps = 0;
float meanfps_static = 0;

int main(int argc, char * argv[])
 {
      ofstream results;
  results.open ("detected_litters.txt");
  results<<"        detected litters \n\n";
       ifstream File( "parameters.txt" );
    static const std::streamsize max = std::numeric_limits<std::streamsize>::max();
std::vector<int> values;
double value;
File.ignore(max, '=');
short f=1;
while(f<=12)
{
    File >> value;
   // values.push_back(value);
        switch (f)
                
                {
            case 1: staticness_th=(double) value;break;
            case 2: objectness_th=(double) value;break;
            case 3: aotime=(int) value;break;
        case 4: aotime2=(int) value;break;
        case 5: alpha=(double) value;break;
        case 6: fore_th=(int) value;break;
        case 7: frameinit=(int) value;break;
        case 8: low_light=(bool) value;break;
        case 9: framemod=(int) value;break;
        case 10: framemod2=(int) value;break;
        case 11: minsize=(int) value;break;
        case 12: resize_scale=(double) value;break;
        
                }   
        File.ignore(max, '=');
        f++;

}
    char * videopath;


    if (argc < 2)
        cout << "please specify the video path" << endl;
    else
        videopath = argv[1];

    VideoCapture capture(videopath);


    Mat fore;
    //capture.set(CV_CAP_PROP_POS_FRAMES, 255);
    capture >> image;

    if (resize_scale != 1)
        resize(image, image, Size(image.cols * resize_scale, image.rows * resize_scale));

  //  image = image(Rect(20, 20, image.cols - 40, image.rows - 40));
    //image = image(Rect(0, 0, image.cols, image.rows - 15));

    //capture.set(CV_CAP_PROP_FPS,60);
    cv::Mat abandoned_map = Mat::zeros(image.size(), CV_8UC1);
    abandoned_map.copyTo(threshed1);
    abandoned_map.copyTo(result);
    abandoned_map.copyTo(object_map);

    Mat segmap1 = Mat::zeros(image.size(), CV_16U);
    Mat dirsum1 = Mat::zeros(image.size(), CV_32F);
dirsum1.copyTo(D_Sx);
dirsum1.copyTo(D_Sy);

    Mat input;
    int i = 0;


    double alpha_S;
    double alpha_init = 0.01;
    alpha_S = alpha_init;
         imshow("gray",image);

      abandoned_map.at<uchar>(abandoned_map.cols,abandoned_map.rows) =255;

  
       uchar* testptr = abandoned_map.ptr<uchar>(0,image.rows);
				testptr=testptr+image.cols;
				cout<<"value derniére "<< (uchar)*testptr <<endl;
				cout<<"value derniére at "<<  (uchar)abandoned_map.at<uchar>(abandoned_map.cols,abandoned_map.rows) <<endl;

				cv::waitKey(0);
			      		uint64_t p=0;
    while (1) {

        
        
        if (i > frameinit) alpha_S = alpha;

        capture >> image;
        if(image.empty())
            break;
      //  image = image(Rect(20, 20, image.cols - 40, image.rows - 40));    

        if (resize_scale != 1)
            resize(image, image, Size(image.cols * resize_scale, image.rows * resize_scale));

      //  image.copyTo(input);

        if (i % framemod != 0 && i > frameinit) {

            i++;
            continue;
        }
        double t = (double) getTickCount();

        cout << "frame " << i << endl;
        cv::Mat F_Sx = Mat::zeros(image.size(), CV_8UC1);
        cv::Mat F_Sy = Mat::zeros(image.size(), CV_8UC1);


        cvtColor(image, gray, CV_BGR2GRAY);
        // imshow("gray",gray);
        blur(gray, gray, Size(3, 3));

        if (low_light)
            gray = gray * 1.5;


        Sobel(gray, grad_x, CV_32F, 1, 0, 3, 1, 0, BORDER_DEFAULT);
        Sobel(gray, grad_y, CV_32F, 0, 1, 3, 1, 0, BORDER_DEFAULT);



        cv::Mat dir;
        cv::cartToPolar(grad_x, grad_y, normm, dir, true);


        if (i == 0) {
            // X direction
            grad_x.copyTo(B_Sx);

            // Y direction
            grad_y.copyTo(B_Sy);


        }

        else {
            
    

            D_Sx = grad_x - B_Sx;
           B_Sx = B_Sx + alpha_S*D_Sx;
     
         
              D_Sy = grad_y - B_Sy;
           B_Sy = B_Sy + alpha_S*D_Sy;
           
       result= result.zeros(image.size(), CV_8UC1);
    threshed1   =threshed1.zeros(image.size(), CV_8UC1);
      object_map = object_map.zeros(image.size(), CV_8UC1);
      
      
    
	unsigned char *abandoned_map_ptr = (unsigned char*)(abandoned_map.data);
	unsigned char *result_ptr = (unsigned char*)(result.data);
	unsigned char *object_map_ptr = (unsigned char*)(object_map.data);
	
	unsigned char *F_Sy_ptr = (unsigned char*)(F_Sy.data);
	unsigned char *F_Sx_ptr = (unsigned char*)(F_Sx.data);

	float  *grad_x_ptr = (float)(grad_x.data);
	float  *grad_y_ptr = (float)(grad_y.data);
	float  *D_Sy_ptr = (float)(D_Sy.data);
	float  *D_Sx_ptr = (float)(D_Sx.data);


			for (int j = 1; j < image.rows - 1; j++) {
               
            	//for (int k = 1; k < image.cols - 1; k++) {
        			for (int i = 1; i < image.cols - 1; i++) {

					
					//cout<<"pixel value"<<(uint)*abandoned_map_ptr<<endl;
                    if (i % framemod2 == 0)
                        if (*abandoned_map_ptr== 1)//&& stat.at<uchar>(j,k)==0)
                            *abandoned_map_ptr -= 1;

                    if (abs(*D_Sx_ptr) > fore_th && abs(*grad_x_ptr) >= 20) 
                    {
						*F_Sx_ptr= 255;
						if(p>100000)
						{
			cout<<"pixel value"<<*D_Sx_ptr<<endl;
	
						cout <<"gggggggggggg"<<endl;
					}
						}
					if (abs(*D_Sy_ptr) > fore_th && abs(*grad_y_ptr) >= 20) *F_Sy_ptr= 255;
					
								// cout<<"Fx value"<<(float)abs(*grad_y_ptr)<<endl;

                    if (i > frameinit && i % framemod2 == 0) {
                        //alpha_S = 0.0005;

                        if (*F_Sx_ptr== 255 || *F_Sy_ptr== 255)//&& abandoned_map.at<uchar>(j,k)<255) 					
                            *abandoned_map_ptr += 2;

                       // for (int r0 = -1; r0 <= 1; r0++)
                        //{
                            for (int c0 = -1; c0 <= 1; c0++) {
                               // int j1 = j + r0;
                                //int k1 = k + c0;
                                // 60-30 PETS 120-80 AVSS
                                
          if ((*abandoned_map_ptr+c0) > aotime && *abandoned_map_ptr > aotime2 && *abandoned_map_ptr < aotime)
                                    *abandoned_map_ptr = aotime;
                                    
 if ((*abandoned_map_ptr_forward+c0) > aotime && *abandoned_map_ptr > aotime2 && *abandoned_map_ptr < aotime)
                                    *abandoned_map_ptr = aotime;                                    
							
 if ((*abandoned_map_ptr_back+c0) > aotime && *abandoned_map_ptr > aotime2 && *abandoned_map_ptr < aotime)
                                    *abandoned_map_ptr = aotime;							
							}
                          //  }
                       //threshed1.at<uchar>(j, k) = 0;
                       //result.at<uchar>(j, k) = 0;
                     //object_map.at<uchar>(j, k) = 0;
                        if (*abandoned_map_ptr > aotime) {
                            *result_ptr = 255;
                            //threshed1.at<uchar>(j, k) = 220;
                        }
                      //  if (abandoned_map.at<uchar>(j, k) > 5 && abandoned_map.at<uchar>(j, k) < aotime)
                           // threshed1.at<uchar>(j, k) = 30;

                        if (*abandoned_map_ptr > aotime2)
                         
                            *object_map_ptr = 255;
                     

                    }
                   abandoned_map_ptr++;
               object_map_ptr++;
				F_Sy_ptr++;
				F_Sx_ptr++;
				grad_x_ptr++;
				grad_y_ptr++;
				result_ptr++;
				D_Sx_ptr++;
				D_Sy_ptr++;
				abandoned_map_ptr_forward++;
				abandoned_map_ptr_back++;
                }
         


            }
                            				cout<<"running   "<<p<<endl;

        double t2 = ((double) getTickCount() - t) / getTickFrequency();
        cout << " static region FPS  " << 1 / t2 << endl;
                meanfps_static = (meanfps_static + (1 / t2)) / 2;

        cout << "mean FPS region  " << meanfps_static << endl;

            F = F_Sx + F_Sy;
            Mat  map2;

          //  bitwise_not(abandoned_map, accumulation);
            abandoned_map.copyTo(map2);
            abandoned_objects.extractObject(result, image, i, abandoned_map, map2);
            cv::Canny(gray, bw, 30, 30 * 3, 3);

            Mat gg;
            // dir1 = dir*(PI / 180);
            cv::cartToPolar(grad_x, grad_y, gg, dir1);
            cv::Mat finalmap1(image.size(), CV_8UC3, Scalar(255, 255, 255));
            finalmap = finalmap1;


            abandoned_objects.processed_objects.clear();
            stop = false;

            bool enter = false;
            for (int u = 0; u < abandoned_objects.abandonnes.size(); u++) {
                bool process = false;
                for (int e = 0; e < abandoned_objects.processed_objects.size(); e++) {

                    if (abs(abandoned_objects.processed_objects[e].origin.x - abandoned_objects.abandonnes[u].origin.x) < 20 && abs(abandoned_objects.processed_objects[e].origin.y - abandoned_objects.abandonnes[u].origin.y) < 20
                            && abs(abandoned_objects.processed_objects[e].endpoint.x - abandoned_objects.abandonnes[u].endpoint.x) < 20 && abs(abandoned_objects.processed_objects[e].endpoint.y - abandoned_objects.abandonnes[u].endpoint.y) < 20) {
                        process = true;

                        break;
                    }
                }
                if (process) continue;

                AO obj;
                obj = abandoned_objects.abandonnes[u];
                abandoned_objects.processed_objects.push_back(obj);
                if (abs(obj.origin.y - obj.endpoint.y) < 15 || abs(obj.origin.x - obj.endpoint.x) < minsize) continue;

                if (obj.origin.y < 6) obj.origin.y = 6;
                if (obj.origin.x < 6) obj.origin.x = 6;
                if (obj.endpoint.y> int(image.rows - 6)) obj.endpoint.y = image.rows - 6;
                if (obj.endpoint.x> int(image.cols - 6)) obj.endpoint.x = image.cols - 6;


              //  rectangle(image, Rect(obj.origin, obj.endpoint), Scalar(255, 255, 255));



                float Staticness, Objectness;

                for (int r0 = -2; r0 <= 2; r0++) {

                    uint y = obj.origin.y + r0;
                    uint x = obj.origin.x + r0;
                    if (obj.origin.y < 5 && obj.origin.x < 5) continue;

                    for (int c0 = -2; c0 <= 2; c0++) {
                        uint w = obj.endpoint.y + c0;
                        uint h = obj.endpoint.x + c0;

						// FIXME: added missing clamping
						if (w < 0) w = 0; if (w > image.cols) w = image.cols;
						if (h < 0) h = 0; if (h > image.rows) h = image.rows;
						
                        segmap1.copyTo(segmap);
                        dirsum1.copyTo(dirsum);
                        Staticness = 0;
                        Objectness = 0;

                        edge_segments(x, y, w, h, Staticness, Objectness);

                        if (Staticness > staticness_th && Objectness > objectness_th && Objectness < 1000000) {
                            enter = true;
                            //results<<" x: "<< x<<" y: "<<y<<" w: "<<w<<" h: "<<h<<endl; 
                            rectangle(image, Rect(obj.origin, obj.endpoint), Scalar(0, 0, 255), 2);
                            //rectangle(threshed1, Rect(obj.origin, obj.endpoint), Scalar(255, 255, 255), 2);

                            //rectangle(map2, Rect(obj.origin, obj.endpoint), Scalar(0, 0, 255), 2);
                            putText(image, "Abandoned !", Point(obj.origin.x, obj.origin.y - 10), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 2, 8, false);
                            abandoned_objects.abandonnes[u].abandoness++;

                            if (abandoned_objects.abandonnes[u].abandoness > 0) {
                                stop = true;
                                abandoned_objects.abandonnes[u].update = false;
                                abandoned_objects.abandonnes[u].activeness = 200;
                            }

                        }
                    }
                }

            }

            cvtColor(threshed1, threshed, CV_GRAY2BGR);
            cvtColor(F, fore, CV_GRAY2BGR);

            bitwise_not(fore, fore);
            bitwise_not(threshed, threshed);
            imshow("output", image);
            imshow("static edges", result);
            imshow("moving edges", fore);
            imshow("finalmap", finalmap);

            waitKey(10);

        }
        t = ((double) getTickCount() - t) / getTickFrequency();

        cout << "FPS  " << 1 / t << endl;
        meanfps = (meanfps + (1 / t)) / 2;
        cout << "mean FPS  " << meanfps << endl;

        i++;
    }
    return 0;
}

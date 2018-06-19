
#include "edge_grouping.h"
#include "scoring.h"
#include "parameters.h"
#include <omp.h>

objects abandoned_objects;

Mat image, gray, F;
Mat grad_x, D_Sx, B_Sx, F_Sx;
Mat grad_y, D_Sy, B_Sy, F_Sy;
Mat result, threshed1, accumulation;
Mat people_heat_map;

int vis;
float meanfps = 0;
float meanfps_static = 0;
static void detectAndDraw(const HOGDescriptor &hog, Mat &img)
{
    vector<Rect> found, found_filtered;
    double t = (double) getTickCount();
    // Run the detector with default parameters. to get a higher hit-rate
    // (and more false alarms, respectively), decrease the hitThreshold and
    // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
    hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 3);
   // t = (double) getTickCount() - t;
    //cout << "detection time = " << (t*1000./cv::getTickFrequency()) << " ms" << endl;

    for(size_t i = 0; i < found.size(); i++ )
    {
        Rect r = found[i];

        size_t j;
        // Do not add small detections inside a bigger detection.
        for ( j = 0; j < found.size(); j++ )
            if ( j != i && (r & found[j]) == r )
                break;

        if ( j == found.size() )
            found_filtered.push_back(r);
    }

    for (size_t i = 0; i < found_filtered.size(); i++)
    {
        Rect r = found_filtered[i];

        // The HOG detector returns slightly larger rectangles than the real objects,
        // so we slightly shrink the rectangles to get a nicer output.
        r.x += cvRound(r.width*0.1);
        r.width = cvRound(r.width*0.8);
        r.y += cvRound(r.height*0.07);
        r.height = cvRound(r.height*0.8);
        rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 3);
        	//ellipse( img, Point( r.x+r.width/2, r.y+r.height ), Size( r.height/2, r.width/2 ), 0, 0, 360, Scalar( 60, 60, 60 ), -1, 8 );
	//ellipse( img, Point( r.x+r.width/2, r.y+r.height ), Size( r.height/2, r.width/2 ), 0, 0, 360, Scalar( 200, 200, 200 ), -1, 8 );

	ellipse( people_heat_map, Point( r.x+r.width/2, r.y+r.height ), Size( r.height/2, r.width/2 ), 0, 0, 360, Scalar( 200, 200, 200 ), -1, 8 );
	
    }
}
int main(int argc, char * argv[])
 {
//    cout<<"optimized ? "<<cv::useOptimized()<<endl;;
 
	     HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
	 
	 
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
        // imshow("gray",image);

     
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

      //  cout << "frame " << i << endl;
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


         if (i % framemod2 == 0)
      abandoned_map-=1;
      
    
	
			for (int j = 1; j < image.rows - 1; j++) {




	    //float* B_Sy_ptr = B_Sy.ptr<float>(j)+1;
	//	    float* B_Sx_ptr = B_Sx.ptr<float>(j)+1;
          // uchar* threshed1_ptr = threshed1.ptr<uchar>(j)+1;

				//pointers declaration
				//abandoned map
				//cout<<"row "<<j<<endl;
                uchar* abandoned_map_ptr = abandoned_map.ptr<uchar>(j)+1;
                 uchar* abandoned_map_ptr_forward = abandoned_map.ptr<uchar>(j+1)+1;
                uchar* abandoned_map_ptr_back = abandoned_map.ptr<uchar>(j-1)+1;

                const uchar* abandoned_map_endPixel = abandoned_map_ptr + abandoned_map.cols -1;
                
                //results
               // uchar* result_ptr = result.ptr<uchar>(j)+1;
               // const uchar* result_endPixel = result_ptr + abandoned_map.cols -1;
                
                        //object_map
                //uchar* object_map_ptr = object_map.ptr<uchar>(j)+1;
                //const uchar* object_map_endPixel = object_map_ptr + abandoned_map.cols-1;
                
                	//F_Sx
                uchar* F_Sx_ptr = F_Sx.ptr<uchar>(j)+1;
                //const uchar* F_Sx_endPixel = F_Sx_ptr + abandoned_map.cols-1;
                
                	//F_Sy
              uchar* F_Sy_ptr = F_Sy.ptr<uchar>(j)+1;
               // const uchar* F_Sy_endPixel = F_Sy_ptr + abandoned_map.cols -1;
                
                	//grad_x
                float* grad_x_ptr = grad_x.ptr<float>(j)+1;
                //const float* grad_x_endPixel = grad_x_ptr + abandoned_map.cols-1;
                
                	//grad_y
              float* grad_y_ptr = grad_y.ptr<float>(j)+1;
               // const float* grad_y_endPixel = grad_y_ptr + abandoned_map.cols-1;

				
				
					//D_Sx
                float* D_Sx_ptr = D_Sx.ptr<float>(j)+1;
                //const float* D_Sx_endPixel = D_Sx_ptr + abandoned_map.cols-1;
                
                	//D_Sy
              float* D_Sy_ptr = D_Sy.ptr<float>(j)+1;
                //const float* D_Sy_endPixel = D_Sy_ptr + abandoned_map.cols-1;

               
            	//for (int k = 1; k < image.cols - 1; k++) {
          while(abandoned_map_ptr != abandoned_map_endPixel) {

//  *D_Sx_ptr=*grad_x_ptr- *B_Sx_ptr;
  //         *B_Sx_ptr=*B_Sx_ptr+alpha_S**D_Sx_ptr;
           
    //         *D_Sy_ptr=*grad_y_ptr- *B_Sy_ptr;
      //     *B_Sy_ptr=*B_Sy_ptr+alpha_S**D_Sy_ptr;
					
					//cout<<"pixel value"<<(uint)*abandoned_map_ptr<<endl;
                 //   if (i % framemod2 == 0)
                   //     if (*abandoned_map_ptr>= 1)//&& stat.at<uchar>(j,k)==0)
                     //       *abandoned_map_ptr -= 1;

                    if (abs(*D_Sx_ptr) > fore_th && abs(*grad_x_ptr) >= 20) *F_Sx_ptr= 255;
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
                        // * result_ptr=0;
                        // *     object_map_ptr=0;
                      // *threshed1_ptr = 0;

                      // threshed1.at<uchar>(j, k) = 0;
                       //result.at<uchar>(j, k) = 0;
                     //object_map.at<uchar>(j, k) = 0;
                  //      if (*abandoned_map_ptr > aotime) {
                    //        *result_ptr = 255;
                            //threshed1.at<uchar>(j, k) = 220;
                      //  }
                      //  if (abandoned_map.at<uchar>(j, k) > 5 && abandoned_map.at<uchar>(j, k) < aotime)
                           // threshed1.at<uchar>(j, k) = 30;

                        //if (*abandoned_map_ptr > aotime2)
                         
                          //  *object_map_ptr = 255;
                     

                    }
                   abandoned_map_ptr++;
               //object_map_ptr++;
				F_Sy_ptr++;
				F_Sx_ptr++;
				grad_x_ptr++;
				grad_y_ptr++;
				//result_ptr++;
				D_Sx_ptr++;
				D_Sy_ptr++;
				abandoned_map_ptr_forward++;
				abandoned_map_ptr_back++;

				//B_Sx_ptr++;
				//B_Sy_ptr++;
                }
         


            }
threshold(abandoned_map,result, aotime, 255, THRESH_BINARY);
		threshold(abandoned_map,object_map, aotime2, 255, THRESH_BINARY);
      double t2 = ((double) getTickCount() - t) / getTickFrequency();
//      cout << " static region FPS  " << 1 / t2 << endl;
              meanfps_static = meanfps_static + (1 / t2);



//            F = F_Sx + F_Sy;
            //Mat  map2;

//           bitwise_not(abandoned_map, accumulation);
//           imshow("accumulation",accumulation);
           // abandoned_map.copyTo(map2);
            abandoned_objects.extractObject(result, image, i, abandoned_map);
            cv::Canny(gray, bw, 30, 30 * 3, 3);

            Mat gg;
             dir1 = dir*(PI / 180);
           // cv::cartToPolar(grad_x, grad_y, gg, dir1);
//            cv::Mat finalmap1(image.size(), CV_8UC3, Scalar(255, 255, 255));
  //       finalmap = finalmap1;
//             finalmap.create(image.size(), CV_8UC3, Scalar(255, 255, 255));
    finalmap.create(image.rows, image.cols, CV_8UC3);
            finalmap=Scalar::all(255);

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


              // rectangle(image, Rect(obj.origin, obj.endpoint), Scalar(255, 255, 255));



                float Staticness, Objectness;

               

                    uint y = obj.origin.y;
                    uint x = obj.origin.x;
                    if (obj.origin.y < 5 && obj.origin.x < 5) continue;

                   
                        uint w = obj.endpoint.y ;
                        uint h = obj.endpoint.x ;

						// FIXME: added missing clamping
					//	if (w < 0) w = 0; if (w > image.cols) w = image.cols;
						//if (h < 0) h = 0; if (h > image.rows) h = image.rows;
						
  //                      segmap1.copyTo(segmap);
    //                  dirsum1.copyTo(dirsum);
                            dirsum.create(image.rows, image.cols, CV_32F);
                        segmap.create(image.rows, image.cols,CV_16U);
						dirsum=Scalar::all(0);
						segmap=Scalar::all(0);
	                    Staticness = 0;
                        Objectness = 0;
			cv::Rect AO_ROI(cv::Point(x, y), cv::Point(h, w));
			int AO_area = cv::countNonZero(people_heat_map(AO_ROI));
			if( AO_area> 400) 
			{
                        edge_segments(y, x, w, h, Staticness, Objectness);

                        if (Staticness > staticness_th && Objectness > objectness_th && Objectness < 1000000) {
                            enter = true;
                            results<<" x: "<< x<<" y: "<<y<<" w: "<<w<<" h: "<<h<<endl; 
                           rectangle(image, Rect(obj.origin, obj.endpoint), Scalar(0, 0, 255), 2);
                            //rectangle(threshed1, Rect(obj.origin, obj.endpoint), Scalar(255, 255, 255), 2);

                            //rectangle(map2, Rect(obj.origin, obj.endpoint), Scalar(0, 0, 255), 2);
                            //putText(image, "Abandoned !", Point(obj.origin.x, obj.origin.y - 10), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 2, 8, false);
                            abandoned_objects.abandonnes[u].abandoness++;

                            if (abandoned_objects.abandonnes[u].abandoness > 0) {
                                stop = true;
                                abandoned_objects.abandonnes[u].update = false;
                                abandoned_objects.abandonnes[u].activeness = 200;
                            }

                        }
			}
                

            }
            detectAndDraw(hog, image);
people_heat_map=people_heat_map-1;

          //  cvtColor(threshed1, threshed, CV_GRAY2BGR);
           // cvtColor(F, fore, CV_GRAY2BGR);

            bitwise_not(fore, fore);
            //bitwise_not(threshed, threshed);
            imshow("output", image);
            //imshow("static edges", result);
         //   imshow("moving edges", fore);
            //imshow("finalmap", finalmap);

            waitKey(10);

        }
        t = ((double) getTickCount() - t) / getTickFrequency();

        cout << "FPS  " << 1 / t << endl;
        meanfps =  (1 / t)+meanfps;

//		cout<<"FPS "<<1/t<<endl;
        i++;
    }
        cout << "mean FPS  " << meanfps/i << endl;
        cout << "mean FPS region  " << meanfps_static/i << endl;

    return 0;
}

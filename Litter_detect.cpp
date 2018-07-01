#include "edge_grouping.h"
#include "scoring.h"
#include "parameters.h"
#include <omp.h>
#include <map>
#include <functional>

//fixme: all those DECLARE_PARAM can be rewritten using C++17 and std::variant
//see here: https://habr.com/post/415737/

//this must be once per program - this allocates actual memory
#define DECLARE_PARAM(TYPE, NAME) TYPE NAME
//important parameters
DECLARE_PARAM(float, staticness_th) ; // Staticness score threshold (degree of the object being static)
DECLARE_PARAM(double, objectness_th); //  Objectness score threshold (probability that the rectangle contain an object)
DECLARE_PARAM(fullbits_int_t, aotime); //aotime*framemod2= number of frames the objects must be  static// if set too low, maybe cause false detections !!
DECLARE_PARAM(fullbits_int_t, aotime2); // Half or more of aotime
DECLARE_PARAM(double, alpha); // background scene learning rate
DECLARE_PARAM(double, fore_th); // Threshold moving edges segmentation


//Less important parameters
DECLARE_PARAM(bool, low_light); // if night scene
DECLARE_PARAM(fullbits_int_t, frameinit); // Frames needed for learning the background
DECLARE_PARAM(fullbits_int_t, framemod);
DECLARE_PARAM(fullbits_int_t, framemod2);
DECLARE_PARAM(fullbits_int_t, minsize); // Static object minimum size
DECLARE_PARAM(float, resize_scale); // Image resize scale
#undef DECLARE_PARAM


//and 1 more copy-paste (MUST BE SAME AS ABOVE!) which defines relation between string-name and variables above
#define DECLARE_PARAM(TYPE, NAME) {#NAME, [](const std::string& src){setParam<TYPE>(src, &NAME);}}
const static std::map<std::string, std::function<void(const std::string& src)>> param_setters =
{
    //important parameters
    DECLARE_PARAM(float, staticness_th), // Staticness score threshold (degree of the object being static)
    DECLARE_PARAM(double, objectness_th), //  Objectness score threshold (probability that the rectangle contain an object)
    DECLARE_PARAM(fullbits_int_t, aotime), //aotime*framemod2= number of frames the objects must be  static// if set too low, maybe cause false detections !!
    DECLARE_PARAM(fullbits_int_t, aotime2), // Half or more of aotime
    DECLARE_PARAM(double, alpha), // background scene learning rate
    DECLARE_PARAM(double, fore_th), // Threshold moving edges segmentation


    //Less important parameters
    DECLARE_PARAM(bool, low_light), // if night scene
    DECLARE_PARAM(fullbits_int_t, frameinit), // Frames needed for learning the background
    DECLARE_PARAM(fullbits_int_t, framemod),
    DECLARE_PARAM(fullbits_int_t, framemod2),
    DECLARE_PARAM(fullbits_int_t, minsize), // Static object minimum size
    DECLARE_PARAM(float, resize_scale), // Image resize scale
};
#undef DECLARE_PARAM

int main(int argc, char * argv[])
{
    using namespace cv;

    float meanfps = 0;
    float meanfps_static = 0;


    std::ofstream results;
    results.open ("detected_litters.txt");
    results << "        detected litters \n\n";
    std::ifstream paramsFile( "parameters.txt" );

    if (!paramsFile.is_open())
    {
        std::cerr << "Cannot load parameters.txt" << std::endl;
        exit(2);
    }

    for (std::string tmp; !paramsFile.eof();)
    {
        std::getline(paramsFile, tmp);
        auto peq = tmp.find_first_of('=');
        if (peq != std::string::npos)
        {
            auto name = trim_copy(tmp.substr(0, peq ));
            auto val  = trim_copy(tmp.substr(peq + 1));
            std::cout << "Parsing: " << name << " = " << val << std::endl;
            if (param_setters.count(name))
                param_setters.at(name)(val);
            else
                std::cerr << "Unknown parameter line: " << tmp << std::endl;
        }
    }
    char * videopath = nullptr;


    if (argc < 2)
    {
        std::cout << "please specify the video path" << std::endl;
        exit(1);
    }
    else
        videopath = argv[1];


    cv::VideoCapture capture(videopath);
    const auto framesCount = static_cast<long>(capture.get(CV_CAP_PROP_FRAME_COUNT));
    capture.set(CV_CAP_PROP_BUFFERSIZE, 1);


    cv::Mat image;
    capture >> image;
    if (image.empty())
    {
        std::cerr << "Something wrong. 1st frame is empty. Cant continue." << std::endl;
        exit(3);
    }

    if (resize_scale != 1)
        resize(image, image, cv::Size(image.cols * resize_scale, image.rows * resize_scale));

    const auto zeroMatrix8U     = cv::Mat::zeros(image.size(), CV_8UC1);
    const auto zeroMatrixSegmap = cv::Mat::zeros(image.size(), getCvTypeFor1Channel<segmap_t>());
    const auto zeroMatrix32F = cv::Mat::zeros(image.size(), CV_32F);
    const auto ffMatrix8UC3  = cv::Mat::ones(image.size(), CV_8UC3) * 255;


    cv::Mat abandoned_map = zeroMatrix8U;

    double alpha_S;
    double alpha_init = 0.01;
    alpha_S = alpha_init;
    // imshow("gray",image);

    objects abandoned_objects(framesCount);
    cv::Mat B_Sx, B_Sy;
    for (fullbits_int_t i = 0; !image.empty(); ++i, (capture >> image))
    {
        if (i > frameinit) alpha_S = alpha;
        if (i % framemod != 0 && i > frameinit)
            continue;

        if (resize_scale != 1)
            resize(image, image, Size(image.cols * resize_scale, image.rows * resize_scale));

        auto t = static_cast<double>(getTickCount());

        //  cout << "frame " << i << endl;
        cv::Mat F_Sx = zeroMatrix8U;
        cv::Mat F_Sy = zeroMatrix8U;

        cv::Mat gray;
        cv::cvtColor(image, gray, CV_BGR2GRAY);
        cv::blur(gray, gray, Size(3, 3));

        if (low_light)
            gray = gray * 1.5;

        cv::Mat grad_x, grad_y;
        cv::Sobel(gray, grad_x, CV_32F, 1, 0, 3, 1, 0, BORDER_DEFAULT);
        cv::Sobel(gray, grad_y, CV_32F, 0, 1, 3, 1, 0, BORDER_DEFAULT);



        cv::Mat dir;
        cv::cartToPolar(grad_x, grad_y, normm, dir, true);

        object_map = zeroMatrix8U;
        if (i == 0)
        {
            // X direction
            grad_x.copyTo(B_Sx);

            // Y direction
            grad_y.copyTo(B_Sy);
        }
        else
        {
            cv::Mat D_Sx = grad_x - B_Sx;
            B_Sx = B_Sx + alpha_S * D_Sx;


            cv::Mat D_Sy = grad_y - B_Sy;
            B_Sy = B_Sy + alpha_S * D_Sy;



            if (i % framemod2 == 0)
                abandoned_map -= 1;



            for (fullbits_int_t j = 1; j < image.rows - 1; ++j)
            {
                uchar* abandoned_map_ptr = abandoned_map.ptr<uchar>(j) + 1;
                uchar* abandoned_map_ptr_forward = abandoned_map.ptr<uchar>(j + 1) + 1;
                uchar* abandoned_map_ptr_back = abandoned_map.ptr<uchar>(j - 1) + 1;

                const uchar* abandoned_map_endPixel = abandoned_map_ptr + abandoned_map.cols - 1;

                //F_Sx
                uchar* F_Sx_ptr = F_Sx.ptr<uchar>(j) + 1;
                //const uchar* F_Sx_endPixel = F_Sx_ptr + abandoned_map.cols-1;

                //F_Sy
                uchar* F_Sy_ptr = F_Sy.ptr<uchar>(j) + 1;
                // const uchar* F_Sy_endPixel = F_Sy_ptr + abandoned_map.cols -1;

                //grad_x
                float* grad_x_ptr = grad_x.ptr<float>(j) + 1;
                //const float* grad_x_endPixel = grad_x_ptr + abandoned_map.cols-1;

                //grad_y
                float* grad_y_ptr = grad_y.ptr<float>(j) + 1;
                // const float* grad_y_endPixel = grad_y_ptr + abandoned_map.cols-1;


                //D_Sx
                float* D_Sx_ptr = D_Sx.ptr<float>(j) + 1;
                //const float* D_Sx_endPixel = D_Sx_ptr + abandoned_map.cols-1;

                //D_Sy
                float* D_Sy_ptr = D_Sy.ptr<float>(j) + 1;
                //const float* D_Sy_endPixel = D_Sy_ptr + abandoned_map.cols-1;


                //for (fullbits_int_t k = 1; k < image.cols - 1; k++) {
                while (abandoned_map_ptr != abandoned_map_endPixel)
                {
                    if (abs(*D_Sx_ptr) > fore_th && abs(*grad_x_ptr) >= 20) *F_Sx_ptr = 255;
                    if (abs(*D_Sy_ptr) > fore_th && abs(*grad_y_ptr) >= 20) *F_Sy_ptr = 255;

                    // cout<<"Fx value"<<(float)abs(*grad_y_ptr)<<endl;

                    if (i > frameinit && i % framemod2 == 0)
                    {
                        //alpha_S = 0.0005;

                        if (*F_Sx_ptr == 255 || *F_Sy_ptr == 255) //&& abandoned_map.at<uchar>(j,k)<255)
                            *abandoned_map_ptr += 2;

                        // for (fullbits_int_t r0 = -1; r0 <= 1; r0++)
                        //{
                        for (fullbits_int_t c0 = -1; c0 <= 1; ++c0)
                        {
                            // fullbits_int_t j1 = j + r0;
                            //fullbits_int_t k1 = k + c0;
                            // 60-30 PETS 120-80 AVSS

                            if ((*abandoned_map_ptr + c0) > aotime && *abandoned_map_ptr > aotime2 && *abandoned_map_ptr < aotime)
                                *abandoned_map_ptr = aotime;

                            if ((*abandoned_map_ptr_forward + c0) > aotime && *abandoned_map_ptr > aotime2 && *abandoned_map_ptr < aotime)
                                *abandoned_map_ptr = aotime;

                            if ((*abandoned_map_ptr_back + c0) > aotime && *abandoned_map_ptr > aotime2 && *abandoned_map_ptr < aotime)
                                *abandoned_map_ptr = aotime;
                        }
                    }
                    ++abandoned_map_ptr;
                    //object_map_ptr++;
                    ++F_Sy_ptr;
                    ++F_Sx_ptr;
                    ++grad_x_ptr;
                    ++grad_y_ptr;
                    //result_ptr++;
                    ++D_Sx_ptr;
                    ++D_Sy_ptr;
                    ++abandoned_map_ptr_forward;
                    ++abandoned_map_ptr_back;
                }
            }
            cv::Mat result     = zeroMatrix8U;
            threshold(abandoned_map, result, aotime, 255, THRESH_BINARY);
            threshold(abandoned_map, object_map, aotime2, 255, THRESH_BINARY);
            double t2 = ((double) getTickCount() - t) / getTickFrequency();
            //      cout << " static region FPS  " << 1 / t2 << endl;
            meanfps_static = meanfps_static + (1 / t2);
            abandoned_objects.extractObject(result, image, i, abandoned_map);
            cv::Canny(gray, bw, 30, 30 * 3, 3);

            constexpr auto pi_180 = (PI) / 180.;
            dir1 = dir * pi_180;
            finalmap = ffMatrix8UC3;


            stop = false;

            sortX(abandoned_objects.abandonnes);

            AO_Collection po;
            po.reserve(abandoned_objects.abandonnes.size());
            for (auto& atu : abandoned_objects.abandonnes)
            {
                const bool process = po.cend() != std::find_if(po.cbegin(), po.cend(), [&atu](const AO & obj)
                {
                    return obj == atu;
                });
                if (process) continue;

                po.push_back(atu);
                sortX(po);

                if (std::abs(atu.origin.y - atu.endpoint.y) < 15 || std::abs(atu.origin.x - atu.endpoint.x) < minsize) continue;

                atu.origin.y = std::max(atu.origin.y, 6);
                atu.origin.x = std::max(atu.origin.x, 6);

                atu.endpoint.y = std::min(atu.endpoint.y, image.rows - 6);
                atu.endpoint.x = std::min(atu.endpoint.x, image.cols - 6);

                float Staticness = 0, Objectness = 0;

                dirsum = zeroMatrix32F;
                segmap = zeroMatrixSegmap;
                //let compiler optimize this hard (added const)
                const auto y = atu.origin.y;
                const auto x = atu.origin.x;
                //dont you think it is suspicous - reverted w/h (and seems reverted again in edge_segment)
                const auto w = atu.endpoint.y ;
                const auto h = atu.endpoint.x ;

                edge_segments(y, x, w, h, Staticness, Objectness);

                if (Staticness > staticness_th && Objectness > objectness_th && Objectness < 1000000)
                {
                    results << " x: " << x << " y: " << y << " w: " << w << " h: " << h << std::endl;
                    const static Scalar color(0, 0, 255);
                    rectangle(image, Rect(atu.origin, atu.endpoint), color, 2);
                    atu.abandoness++;

                    if (atu.abandoness > 0)
                    {
                        stop = true;
                        atu.update = false;
                        atu.activeness = 200;
                    }
                }
            }
            //ok,those 2 take around -5 fps on i7
            imshow("output", image);
            waitKey(10);
        }
        t = ((double) getTickCount() - t) / getTickFrequency();
        meanfps =  (1 / t) + meanfps;
        if (i % 50 == 0 )
            std::cout << "FPS  " << meanfps / (i + 1) << std::endl;
    }
    //std::cout << "mean FPS  " << meanfps / i << std::endl;
    //std::cout << "mean FPS region  " << meanfps_static / i  << std::endl;

    return 0;
}

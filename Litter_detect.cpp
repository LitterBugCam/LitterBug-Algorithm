#include "edge_grouping.h"
#include "scoring.h"
#include "parameters.h"
#include <omp.h>
#include <map>
#include <functional>

//fixme: all those DECLARE_PARAM can be rewritten using C++17 and std::variant
//see here: https://habr.com/post/415737/

//this must be once per program - this allocates actual memory
#define DECLARE_PARAM(TYPE, NAME) TYPE NAME = 0
//important parameters
DECLARE_PARAM(float, staticness_th) ; // Staticness score threshold (degree of the object being static)
DECLARE_PARAM(double, objectness_th); //  Objectness score threshold (probability that the rectangle contain an object)
DECLARE_PARAM(uint8_t, aotime); //aotime*framemod2= number of frames the objects must be  static// if set too low, maybe cause false detections !!
DECLARE_PARAM(uint8_t, aotime2); // Half or more of aotime
DECLARE_PARAM(double, alpha); // background scene learning rate
DECLARE_PARAM(double, fore_th); // Threshold moving edges segmentation


//Less important parameters
DECLARE_PARAM(bool, low_light); // if night scene
DECLARE_PARAM(fullbits_int_t, frameinit); // Frames needed for learning the background
DECLARE_PARAM(fullbits_int_t, framemod);
DECLARE_PARAM(fullbits_int_t, framemod2);
DECLARE_PARAM(fullbits_int_t, minsize); // Static object minimum size
DECLARE_PARAM(float, resize_scale); // Image resize scale
DECLARE_PARAM(float, fps_life); // activeness of candidates = fps * fps_life
#undef DECLARE_PARAM


//and 1 more copy-paste (MUST BE SAME AS ABOVE!) which defines relation between string-name and variables above
#define DECLARE_PARAM(TYPE, NAME) {#NAME, [](const std::string& src){setParam<TYPE>(src, &NAME);}}
const static std::map<std::string, std::function<void(const std::string& src)>> param_setters =
{
    //important parameters
    DECLARE_PARAM(float, staticness_th), // Staticness score threshold (degree of the object being static)
    DECLARE_PARAM(double, objectness_th), //  Objectness score threshold (probability that the rectangle contain an object)
    DECLARE_PARAM(uint8_t, aotime), //aotime*framemod2= number of frames the objects must be  static// if set too low, maybe cause false detections !!
    DECLARE_PARAM(uint8_t, aotime2), // Half or more of aotime
    DECLARE_PARAM(double, alpha), // background scene learning rate
    DECLARE_PARAM(double, fore_th), // Threshold moving edges segmentation


    //Less important parameters
    DECLARE_PARAM(bool, low_light), // if night scene
    DECLARE_PARAM(fullbits_int_t, frameinit), // Frames needed for learning the background
    DECLARE_PARAM(fullbits_int_t, framemod),
    DECLARE_PARAM(fullbits_int_t, framemod2),
    DECLARE_PARAM(fullbits_int_t, minsize), // Static object minimum size
    DECLARE_PARAM(float, resize_scale), // Image resize scale
    DECLARE_PARAM(float, fps_life), // activeness of candidates = fps * fps_life
};
#undef DECLARE_PARAM

struct UniqueIndex
{
    size_t current{0};
    size_t operator()()
    {
        return current++;
    }
};


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

    if (std::abs(fps_life) < 0.00001f)
        fps_life = 40. / 25.; //thats is default test value for my test video

    if (aotime <= aotime2)
    {
        std::cerr << "aotime must be greater then aotime2" << std::endl;
        exit(5);
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
    const auto fps         = std::max(1l, static_cast<long>(capture.get(CV_CAP_PROP_FPS)));
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
    const auto ffMatrix8UC3     = cv::Mat::ones(image.size(), CV_8UC3) * 255;

    cv::Mat abandoned_map = zeroMatrix8U;


    const static double alpha_init = 0.01;
    double alpha_S = alpha_init;

    //2nd param shows how long object should live initially (it was 40 by default)
    //however, lets say 1 second of real time video
    objects abandoned_objects(framesCount, fps * std::max(fps_life, 0.01f));

    cv::Mat B_Sx, B_Sy;
    cv::Mat grad_x, grad_y;
    cv::Mat gray;
    cv::Mat D_Sx, D_Sy;
    cv::Mat frame;
    ZeroedArray<uint8_t> canny(0);
    ZeroedArray<uint8_t> object_map(0);
    ZeroedArray<float> angles(0);
    cv::Mat not_used;
    for (fullbits_int_t i = 0; !image.empty(); ++i, (capture >> image))
    {
        auto t = static_cast<double>(getTickCount());

        if (i > frameinit) alpha_S = alpha;
        if (i % framemod != 0 && i > frameinit)
            continue;

        if (resize_scale != 1)
            resize(image, image, Size(image.cols * resize_scale, image.rows * resize_scale));


        cv::cvtColor(image, gray, CV_BGR2GRAY);
        cv::blur(gray, gray, Size(3, 3));

        if (low_light)
            gray = gray * 1.5;


        cv::Sobel(gray, grad_x, CV_32F, 1, 0, 3, 1, 0, BORDER_DEFAULT);
        cv::Sobel(gray, grad_y, CV_32F, 0, 1, 3, 1, 0, BORDER_DEFAULT);


        if (i == 0)
        {
            // X direction
            grad_x.copyTo(B_Sx);

            // Y direction
            grad_y.copyTo(B_Sy);
        }
        else
        {
            D_Sx = grad_x - B_Sx;
            B_Sx = B_Sx + alpha_S * D_Sx;

            D_Sy = grad_y - B_Sy;
            B_Sy = B_Sy + alpha_S * D_Sy;

            if (i % framemod2 == 0)
            {
                auto plain_map_ptr = abandoned_map.ptr<uchar>();
                for (size_t i = 0, sz = image.rows * image.cols; i < sz; ++i)
                    if (*(plain_map_ptr + i)) //overflow prot
                        *(plain_map_ptr + i) -= 1;

                if (i > frameinit)
                {
                    assert(abandoned_map.isContinuous());
                    assert(grad_x.isContinuous());
                    assert(grad_y.isContinuous());
                    assert(D_Sx.isContinuous());
                    assert(D_Sy.isContinuous());


                    //pointers to the [1st] pixel in the row (0th will be used later in loops as -1)
                    auto abandoned_map_ptr = abandoned_map.ptr<uchar>(1, 1);
                    auto grad_x_ptr = grad_x.ptr<float>(1, 1);
                    auto grad_y_ptr = grad_y.ptr<float>(1, 1);
                    auto D_Sx_ptr = D_Sx.ptr<float>(1, 1);
                    auto D_Sy_ptr = D_Sy.ptr<float>(1, 1);

                    for (fullbits_int_t j = 1; j < image.rows - 1; ++j)
                    {
                        for (fullbits_int_t k = 1; k < image.cols - 1; ++k)
                        {
                            auto *point = abandoned_map_ptr + k;

                            //prevening overflow here
                            //btw original code COULD overflow on whites...
                            if ((std::abs(*(D_Sx_ptr + k)) > fore_th && std::abs(*(grad_x_ptr + k)) >= 20) ||
                                    (std::abs(*(D_Sy_ptr + k)) > fore_th && std::abs(*(grad_y_ptr + k)) >= 20))
                                *point = static_cast<std::remove_pointer<decltype(point)>::type>(std::min(2 + *point, static_cast<int>(255)));


                            if (*point > aotime2 && *point < aotime)
                                for (fullbits_int_t c0 = -1; c0 <= 1; ++c0)
                                {
                                    if (c0 && *(point + c0) > aotime) //excluding c0 = 0 which is meself
                                    {
                                        *point = aotime;
                                        break;
                                    }

                                    if (*(point + image.cols + c0) > aotime )
                                    {
                                        *point = aotime;
                                        break;
                                    }

                                    if (*(point - image.cols + c0) > aotime )
                                    {
                                        *point = aotime;
                                        break;
                                    }
                                }
                        }

                        grad_x_ptr        += image.cols;
                        grad_y_ptr        += image.cols;
                        D_Sx_ptr          += image.cols;
                        D_Sy_ptr          += image.cols;
                        abandoned_map_ptr += image.cols;
                    }
                }
            }

            threshold(abandoned_map, frame, aotime, 255, THRESH_BINARY);

            double t2 = ((double) getTickCount() - t) / getTickFrequency();
            meanfps_static = meanfps_static + (1 / t2);
            abandoned_objects.populateObjects(frame, i);

            cv::Canny(gray, canny.getStorage(), 30, 30 * 3, 3);
            threshold(abandoned_map, object_map.getStorage(), aotime2, 255, THRESH_BINARY);
            cv::cartToPolar(grad_x, grad_y, not_used, angles.getStorage(), false);

            for (auto& atu : abandoned_objects.candidat)
            {
                if (!atu.isTooSmall(minsize))
                {
                    es_param_t params = atu.getScoreParams(image.rows, image.cols);
                    edge_segments(object_map, angles, canny, params);
                    if (params.score > staticness_th && params.circularity > objectness_th && params.circularity < 1000000)
                    {
                        //hm, lets do cheat, if we display object then +1 to life
                        atu.extraLife();
                        results << " x: " << params.rr << " y: " << params.cc << " w: " << params.w << " h: " << params.h << std::endl;
                        const static Scalar color(0, 0, 255);
                        rectangle(image, Rect(atu.origin, atu.endpoint), color, 2);
                    }
                }
            }

            //std::cout << "Objects count: " << ", candidate=" << abandoned_objects.candidat.size() << std::endl;
            std::string text = "FPS: " + std::to_string(meanfps / (i + 1)) + ", candidats count: " + std::to_string(abandoned_objects.candidat.size());
            cv::putText(image,
                        text,
                        cv::Point(5, 20), // Coordinates
                        cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                        1.0, // Scale. 2.0 = 2x bigger
                        cv::Scalar(255, 255, 255), // BGR Color
                        1, // Line Thickness
                        CV_AA); // Anti-alias

            imshow("output", image);//ok,those 2 take around -5 fps on i7
            waitKey(10);
        }
        t = ((double) getTickCount() - t) / getTickFrequency();
        meanfps =  (1 / t) + meanfps;
        //        if (i % 50 == 0 )
        //            std::cout << "FPS  " << meanfps / (i + 1) << ", Objects: " << abandoned_objects.candidat.size() << std::endl;
    }
    //std::cout << "mean FPS  " << meanfps / i << std::endl;
    //std::cout << "mean FPS region  " << meanfps_static / i  << std::endl;

    return 0;
}

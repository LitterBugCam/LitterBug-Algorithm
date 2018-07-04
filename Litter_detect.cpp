#include "edge_grouping.h"
#include "scoring.h"
#include "parameters.h"
#include <map>
#include <functional>

#ifndef NO_GUI
    #include <opencv/highgui.h>
#endif

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

#ifdef USE_GPU
#include "QPULib.h"

//GPU cannot do division, so have to prepare values on CPU
//Float t0 = (xOld < 1.0f) ? xOld : 1.0f / x;
void kernel_atan(Int n, Ptr<Float> x, Ptr<Float> t0_p)
{
    //https://seblagarde.wordpress.com/2014/12/01/inverse-trigonometric-functions-gpu-optimization-for-amd-gcn-architecture/
    Int inc = numQPUs() << 4;
    Ptr<Float> p = x + index() + (me() << 4);
    Ptr<Float> b = t0_p + index() + (me() << 4);
    gather(p);
    gather(b);

    Float xOld;
    Float res;
    Float t0;
    For (Int i = 0, i < n, i = i + inc)
    gather(p + inc);
    gather(b + inc);
    receive(xOld);
    receive(t0);

    Float t1 = t0 * t0;
    Float poly = 0.0872929f;
    poly = -0.301895f + poly * t1;
    poly = 1.0f + poly * t1;
    poly = poly * t0;

    Where(xOld < 1.0f)
    res = poly;
    End

    Where(xOld >= 1.0f)
    res = HALF_PI - poly;
    End

    Where (xOld < 0)
    res = 0 - res;
    End

    store(res, p);
    p = p + inc;
    End

    // Discard pre-fetched vectors from final iteration
    receive(xOld);
}

#endif

int main(int argc, char * argv[])
{
#ifdef USE_GPU
    auto k_atan = compile(kernel_atan);
    k_atan.setNumQPUs(12); //it has 12 units
#endif
    using namespace cv;

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

#ifndef NO_FPS
    std::cout << "Using FPS" << std::endl;
#else
    std::cout << "NOT using FPS" << std::endl;
#endif

#ifndef NO_GUI
    std::cout << "Using GUI" << std::endl;
#else
    std::cout << "NOT using GUI" << std::endl;
#endif
    std::cout.flush();



    const char * videopath = (argc < 2) ? nullptr : argv[1];


    if (!videopath)
    {
        std::cout << "please specify the video path" << std::endl;
        exit(1);
    }
    std::cout << "Video file: " << videopath << std::endl;
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

#ifndef NO_FPS
    float meanfps = 0;
#endif


    for (fullbits_int_t i = 0; !image.empty(); ++i, (capture >> image))
    {
        const size_t pixels_size    = image.cols * image.rows;
        const size_t pixels_size_al = pixels_size + (16 - (pixels_size % 16)); //aligned to 16 items for gpu
#ifndef NO_FPS
        auto t = static_cast<double>(getTickCount());
#endif
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

            assert(grad_x.isContinuous());
            assert(grad_y.isContinuous());
            auto grad_x_ptr = grad_x.ptr<float>();
            auto grad_y_ptr = grad_y.ptr<float>();

            if (i % framemod2 == 0)
            {
                assert(abandoned_map.isContinuous());

                assert(D_Sx.isContinuous());
                assert(D_Sy.isContinuous());

                auto plain_map_ptr = abandoned_map.ptr<uchar>();
                auto D_Sx_ptr = D_Sx.ptr<float>();
                auto D_Sy_ptr = D_Sy.ptr<float>();

                for (size_t k = 0; k < pixels_size; ++k)
                {
                    auto point = plain_map_ptr + k;
                    if (*point) //overflow prot
                        *point -= 1;

                    //prevening overflow here
                    //btw original code COULD overflow on whites...
                    if ((std::abs(*(D_Sx_ptr + k)) > fore_th && std::abs(*(grad_x_ptr + k)) > 19) ||
                            (std::abs(*(D_Sy_ptr + k)) > fore_th && std::abs(*(grad_y_ptr + k)) > 19))
                    {
                        if (*point < 254)
                            *point += 2;
                        else
                            *point = 255;
                    }
                }
                if (i > frameinit)
                {
                    for (fullbits_int_t j = 1; j < image.rows - 1; ++j)
                    {
                        plain_map_ptr += image.cols;
                        for (fullbits_int_t k = 1; k < image.cols - 1; ++k)
                        {
                            auto point = plain_map_ptr + k;

                            //hmm, this code can be removed for test image - same result
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
                    }
                }
            }

            threshold(abandoned_map, frame, aotime, 255, THRESH_BINARY);
            abandoned_objects.populateObjects(frame, i);

            cv::Canny(gray, canny.getStorage(), 30, 30 * 3, 3);
            threshold(abandoned_map, object_map.getStorage(), aotime2, 255, THRESH_BINARY);
#ifdef USE_GPU
            SharedArray<float> yx(pixels_size_al);
            SharedArray<float> t0(pixels_size_al);
            angles.resize(image.rows, image.cols);
            for (size_t k = 0, sz = pixels_size; k < sz; ++k)
            {
                //GPU do not have division...
                *(yx.getPointer() + k) = *(grad_y_ptr + k) / *(grad_x_ptr + k);
                *(t0.getPointer() + k) = *(yx.getPointer() + k);
                if (!(*(t0.getPointer() + k) < 1.f))
                    *(t0.getPointer() + k) = 1.f / *(t0.getPointer() + k);
            }

            k_atan(static_cast<int>(pixels_size), &yx, &t0);
            for (size_t k = 0, sz = pixels_size; k < sz; ++k)
                *(angles.ptr() + k) = *(yx.getPointer() + k);
#else
            cv::cartToPolar(grad_x, grad_y, not_used, angles.getStorage(), false);
#endif


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
#ifndef NO_GUI
                        const static Scalar color(0, 0, 255);
                        rectangle(image, Rect(atu.origin, atu.endpoint), color, 2);
#endif
                    }
                }
            }
#ifndef NO_GUI
#ifndef NO_FPS
            const std::string text = "FPS: " + std::to_string(meanfps / (i + 1)) + ", candidats count: " + std::to_string(abandoned_objects.candidat.size());
#else
            const std::string text = "Candidats count: " + std::to_string(abandoned_objects.candidat.size());
#endif
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
#endif
        }

#ifndef NO_FPS
        t = ((double) getTickCount() - t) / getTickFrequency();
        meanfps =  (1 / t) + meanfps;
        //print out only with no gui
#ifdef NO_GUI
        if (i % 50 == 0 )
            std::cerr << "FPS  " << meanfps / (i + 1) << ", Objects: " << abandoned_objects.candidat.size() << std::endl;
#endif
#endif
    }
    return 0;
}

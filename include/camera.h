#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "ImageProcessor.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <vector>

class Camera{
public:
    Camera(std::string cam_serial_, float frame_rate_, bool show_img_, int img_size_mode_);
    ~Camera();


    cv::Mat acquire_image(double & time, bool & img_ok);

private:
    Spinnaker::CameraPtr cam;
    Spinnaker::CameraList camList;
    Spinnaker::SystemPtr system;
    bool camera_ready;
    bool show_img;
    float frame_rate;
    int img_size_mode;
    std::string cam_serial;
    void set_camera();
    void show_image(cv::Mat & img);
    void print_spinnaker_version();


};
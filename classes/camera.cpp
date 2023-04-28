#include "camera.h"
#include <ros/ros.h>

Camera::Camera(std::string cam_serial_, float frame_rate_, bool show_img_, int img_size_mode_): 
                cam_serial(cam_serial_), frame_rate(frame_rate_), show_img(show_img_), img_size_mode(img_size_mode_){
    system = Spinnaker::System::GetInstance();
    camera_ready = false;
    cam = nullptr;
    set_camera();
}
Camera::~Camera(){

    if(camera_ready){
        cam->EndAcquisition();
        cam->DeInit();
    }
    camList.Clear();

}

void Camera::print_spinnaker_version(){

    const Spinnaker::LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    std::cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
         << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << std::endl
         << std::endl;
}


void Camera::set_camera(){
    camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();
    std::cout<<"Number of cameras detected: "<<numCameras<<std::endl;

    cam = camList.GetBySerial(cam_serial);


    if(!cam.IsValid()) {
        camera_ready = false;
    } else {
        camera_ready = true;

        cam->DeInit();
        cam->Init();

        // cam->LineSelector.SetValue(Spinnaker::LineSelector_Line2);
        // cam->V3_3Enable.SetValue(true);

        cam->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);

        if(cam->DecimationVertical.GetValue()!= 2){
            cam->DecimationVertical.SetValue(2);
        }
        if(cam->DecimationHorizontal.GetValue()!= 2){
            cam->DecimationHorizontal.SetValue(2);
        }

        Spinnaker::GenApi::CBooleanPtr ptrHandlingAcqFrameRateEnable =  cam->GetNodeMap().GetNode("AcquisitionFrameRateEnable");

        if (!IsAvailable(ptrHandlingAcqFrameRateEnable) || !IsWritable(ptrHandlingAcqFrameRateEnable))
        {
            std::cout << "Camera1 "<< " Unable to enable Acquisition Frame Rate (node retrieval). Aborting..."
                    << std::endl;
        }

        // Enable  Acquisition Frame Rate Enable
        ptrHandlingAcqFrameRateEnable->SetValue(true);

        Spinnaker::GenApi::CFloatPtr ptrFrameRate = cam->GetNodeMap().GetNode("AcquisitionFrameRate");
        if (!IsAvailable(ptrFrameRate) || !IsWritable(ptrFrameRate))
        {
            std::cout << "Camera Unable to set Acquisition Frame Rate (node retrieval). Aborting..." << std::endl;
        }

        // Set 10fps for this example
        const float frameRate = frame_rate;
        ptrFrameRate->SetValue(frameRate);
        std::cout << "Camera Frame rate is set to " << frameRate << std::endl;

        Spinnaker::GenApi::INodeMap& sNodeMap = cam->GetTLStreamNodeMap();

        Spinnaker::GenApi::CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
        if (!IsAvailable(ptrHandlingMode) || !IsWritable(ptrHandlingMode))
        {
            std::cout << "Unable to set Buffer Handling mode (node retrieval). Aborting..." << std::endl;
        }

        Spinnaker::GenApi::CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetCurrentEntry();
        if (!IsAvailable(ptrHandlingModeEntry) || !IsReadable(ptrHandlingModeEntry))
        {
            std::cout << "Unable to set Buffer Handling mode (Entry retrieval). Aborting..." << std::endl;
        }

        ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("NewestOnly");
        ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());
        std::cout << "Buffer Handling Mode has been set to " << ptrHandlingModeEntry->GetDisplayName() << std::endl;

        cam->BeginAcquisition();
    }
}


cv::Mat Camera::acquire_image(double & time, bool & img_ok){

    cv::Mat image;
    if(!camera_ready){
        std::cout<<"Camera is not ready"<<std::endl;
        img_ok = false;
        return image;
    }
    img_ok = true;

    Spinnaker::ImagePtr img = cam->GetNextImage();

    if (img->IsIncomplete())
    {
        std::cout << "Image incomplete: " << Spinnaker::Image::GetImageStatusDescription(img->GetImageStatus())<< std::endl;
        return image;
    }

    time = ros::Time::now().toSec();

    const size_t width = img->GetWidth();
    const size_t height = img->GetHeight();

    Spinnaker::ImageProcessor imgproc;

    Spinnaker::ImagePtr convertedImage = imgproc.Convert(img, Spinnaker::PixelFormat_BGR8);
    cv::Mat imgMat = cv::Mat(cv::Size(width, height), CV_8UC3, convertedImage->GetData());


    if(img_size_mode == 2) {
        cv::resize(imgMat, imgMat, imgMat.size()/2);
    } else if (img_size_mode == 3) {
        cv::resize(imgMat, imgMat, imgMat.size()/4);
    }

    image = imgMat.clone();

    img->Release();
    if(show_img) {
        show_image(image);
    }

    return image;
}

void Camera::show_image(cv::Mat & img){
    imshow("asdf", img);
    cv::waitKey(1);
}
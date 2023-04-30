#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>

#include <image_transport/image_transport.h>
#include "camera.h"

bool shotCommand;

void OnSubscribeKeyboardEvent(const std_msgs::BoolConstPtr & msg) {
    if(msg->data) {
        shotCommand = true;
    }
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "still_image_node");
    ros::NodeHandle nh;

    std::string cam_serial;
    nh.param<std::string>("/camera/serial_num", cam_serial, "23195663");

    float frame_rate;
    nh.param<float>("/camera/frame_rate", frame_rate, 60.0);

    bool show_img;
    nh.param<bool>("/camera/show_image", show_img, true);

    int img_size_mode;
    nh.param<int>("/camera/image_size_mode", img_size_mode, 1);

    bool upsidedown;
    nh.param<bool>("/camera/upsidedown", upsidedown, false);

    std::string image_dir;
    nh.param<std::string>("/camera/image_dir", image_dir, "/home/morin/calib_data/");

    Camera cam(cam_serial, frame_rate, show_img, img_size_mode, upsidedown);
    ros::Subscriber subKeyboard = nh.subscribe<std_msgs::Bool>("/keyboard_events", 1, OnSubscribeKeyboardEvent);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image",1 );

    cv_bridge::CvImage img_bridge;
    std_msgs::Header header;

    int saved_img_no = 0;

    ros::Rate loop_rate(100);
    int count = 0;
    shotCommand = false;
    while(ros::ok()){
        bool img_ok = false;
        double time;
        cv::Mat acquired_image =  cam.acquire_image(time, img_ok);
        if(img_ok) {
            header.seq = count;
            header.stamp.fromSec(time);

            if(shotCommand) {
                char data_name[256];
                sprintf(data_name, "%06d", saved_img_no);
                std::string img_path = image_dir + std::string(data_name) + ".png";
                cv::imwrite(img_path, acquired_image);
                printf("Saved image %d\n", saved_img_no);

                ++saved_img_no;
                shotCommand = false;
            }
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", acquired_image).toImageMsg();
            pub.publish(msg);
            ROS_INFO("image sent!");

            ++count;
        }
        ros::spinOnce();
    }
    return 0;
}
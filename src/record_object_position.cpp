//subscribe to rgb input and look in the incoming stream for a a certain object (baxter gripper)
#include <ros/ros.h>
#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <aruco/dictionary.h>
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>

#include <sensor_msgs/image_encodings.h>
#include <baxter_core_msgs/DigitalIOState.h>

#include <image_processing/DescriptorExtraction.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace cv;

Mat pic;
sensor_msgs::ImageConstPtr rgb_msg;
sensor_msgs::CameraInfoConstPtr info_msg;
float my_x,my_y;
double xc = 0.0, yc = 0.0, zc = 0.0;
aruco::CameraParameters camera_char;
vector<aruco::Marker> markers;
int8_t pressed;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    rgb_msg = msg;
    namedWindow("ShowMarker",CV_WINDOW_AUTOSIZE);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    pic = cv_ptr->image;

    aruco::MarkerDetector my_detector;
    my_detector.setDictionary("ARUCO");
    my_detector.detect(pic,markers,camera_char,0.04);
    if (!markers.empty()){
        markers[0].draw(pic,cv::Scalar(94.0, 206.0, 165.0, 0.0));
        markers[0].calculateExtrinsics(0.04,camera_char,false);
        my_x = (int) markers[0][0].x;
        my_y = (int) markers[0][0].y;
        circle(pic, cv::Point(markers[0][0].x, markers[0][0].y), 10, CV_RGB(255,0,0));
    }
    imshow("ShowMarker", pic);
    waitKey(1);
}

void infoimageCb(const sensor_msgs::CameraInfoConstPtr msg){
    info_msg = msg;
}

void depthimageCb(const sensor_msgs::ImageConstPtr& depth_msg){
    if(!pic.empty() && !markers.empty()){
        rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, rgb_msg, info_msg);
        sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
        image_processing::PointCloudT::Ptr input_cloud(new image_processing::PointCloudT);

        pcl::fromROSMsg(ptcl_msg, *input_cloud);
        image_processing::PointT pt = input_cloud->at((int) my_x+(int) my_y*input_cloud->width);
        zc = pt.z;
        xc = pt.x;
        yc = pt.y;
    }
}

void left_cuf_Callback(baxter_core_msgs::DigitalIOState l_cuf_feedbcak){
    pressed = l_cuf_feedbcak.state;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "record_object_position");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);
    camera_char.readFromXMLFile("/home/mukhtar/git/catkin_ws/src/automatic_camera_robot_cal/data/camera_param_baxter.xml");
    image_transport::Subscriber in_image = it_.subscribe("/camera/rgb/image_rect_color", 1, imageCb);
    ros::Subscriber in_info_image = node.subscribe<sensor_msgs::CameraInfoConstPtr>("/camera/rgb/camera_info", 1, infoimageCb);
    image_transport::Subscriber in_depth_image = it_.subscribe("/camera/depth_registered/sw_registered/image_rect", 1, depthimageCb);
    ros::Subscriber sub_l_cuf_msg = node.subscribe<baxter_core_msgs::DigitalIOState>("/robot/digital_io/left_lower_cuff/state", 10, left_cuf_Callback);

    std::ofstream file("./object_positions");

    ros::AsyncSpinner my_spinner(4);
    my_spinner.start();

    //record marker position (object position) if changed in the specified file
    Eigen::Vector3d old_values;
    pressed = false;
    bool release = true, toggle = false;
    double epsilon = 0.01;
    if(file.is_open()){
        ros::Rate rate(50.0);
        rate.sleep();
        std::cout << "go go go" << std::endl;
        while (ros::ok()){
            if(pressed){
                if(release){
                    release = false;
                    toggle = true;
                }
                while(xc != xc || yc != yc || zc != zc);
                old_values << xc, yc, zc;
                Eigen::Vector3d current_values;
                current_values << xc, yc, zc;
                if((current_values - old_values).norm() > epsilon)
                    file << current_values(0) << "," << current_values(1) << "," << current_values(2);
            }
            else
                release = true;
            if(toggle && !pressed){
                file << "\n";
                toggle = false;
            }
            rate.sleep();
        }
    }
    return 0;
}

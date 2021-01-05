#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

// see also http://docs.ros.org/en/jade/api/image_proc/html/rectify_8cpp_source.html

image_transport::CameraSubscriber sub;
image_transport::Publisher pub;
image_geometry::PinholeCameraModel model;

void undistortCallback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
    
    model.fromCameraInfo(info_msg);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::YUV422); // copy image
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat rect;

    model.rectifyImage(cv_ptr->image, rect);

    sensor_msgs::ImagePtr rect_msg = cv_bridge::CvImage(img_msg->header, img_msg->encoding, rect).toImageMsg();
    pub.publish(rect_msg);
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "undistort_image_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    sub = it.subscribeCamera("image", 1, undistortCallback);
    pub = it.advertise("image_undistorted", 1);

    ros::spin();

    return 0;
}

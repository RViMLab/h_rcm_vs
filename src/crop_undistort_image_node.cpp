#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

// we implement this node as image_proc as poor support for rectifying a cropped image,
// which often leads to errors
// see also http://docs.ros.org/en/jade/api/image_proc/html/rectify_8cpp_source.html

image_transport::CameraSubscriber sub;
image_transport::Publisher pub;
image_geometry::PinholeCameraModel model;

int x_offset, y_offtset, width, height;

void undistortCallback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
    
    model.fromCameraInfo(info_msg);

    auto img = cv_bridge::toCvShare(img_msg)->image;

    cv::Mat rect;
    cv::Rect roi(x_offset, y_offtset, width, height);

    model.rectifyImage(img(roi), rect);

    sensor_msgs::ImagePtr rect_msg = cv_bridge::CvImage(img_msg->header, img_msg->encoding, rect).toImageMsg();
    pub.publish(rect_msg);
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "crop_undistort_image_node");
    ros::NodeHandle nh;

    nh.getParam("crop_undistort_image_node/x_offset", x_offset);
    nh.getParam("crop_undistort_image_node/y_offtset", y_offtset);
    nh.getParam("crop_undistort_image_node/width", width);
    nh.getParam("crop_undistort_image_node/height", height);

    image_transport::ImageTransport it(nh);
    sub = it.subscribeCamera("image", 1, undistortCallback);
    pub = it.advertise("image_undistorted", 1);

    ros::spin();

    return 0;
}

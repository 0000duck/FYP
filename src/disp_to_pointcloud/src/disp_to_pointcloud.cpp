#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher PointCloud_pub;
ros::Subscriber depth_sub;
//Mat Q_mat = (Mat_<float>(4,4) << 1, 0, 0, -407.9482955932617, 0, 1, 0, -261.658447265625,  0, 0, 0, 317.4728399853731,  0, 0, 7.566847092656309, -0);
Mat dispImg;


float baseLine=0.1335;
float fx_l = 440.301091, fy_l = 440.301091;
float cx_l = 384.052727;
float cy_l = 259.070488;

float point_x, point_y, point_z;

void rcvDepthCallback(const sensor_msgs::Image msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


   cv::Mat imageDepth_ = cv_ptr->image.clone();


    PointCloud::Ptr PointCloud_msg (new PointCloud);
    PointCloud_msg->header.frame_id = "map";
    PointCloud_msg->height = imageDepth_.rows;
    PointCloud_msg->width  = imageDepth_.cols;

    for(int u = 0; u < imageDepth_.rows; u++)
        for(int v = 0; v < imageDepth_.cols; v++)
        {

            point_z = imageDepth_.at<float>(u, v);
            
            if(point_z <= 0.0 || isnan(point_z) || point_z > 10.0 ) {
              PointCloud_msg->points.push_back(pcl::PointXYZ(0, 0, 0));
              continue;
            }

	    point_x = (v - cx_l) * point_z / fx_l;
	    point_y = (u - cy_l) * point_z / fy_l;

/*
            point_x = u;
            point_y = v;
            point_z = imageDepth_.at<float>(u, v);
            if(point_z <= 0.0 || isnan(point_z) || point_z > 10.0 ) {
              PointCloud_msg->points.push_back(pcl::PointXYZ(0, 0, 0));
              continue;
            }
*/
            //cout << "u " << u << " v " << v << " " << point_x/100 << " " << point_y/100 << " " << point_z << endl;
            PointCloud_msg->points.push_back(pcl::PointXYZ(point_x, point_y, point_z));

        }

    PointCloud_pub.publish(PointCloud_msg);
}

/*
void image_pointcloud_convert(const sensor_msgs::ImageConstPtr& image_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Mat disp_im = cv_ptr->image.clone();


    imshow("disp_im", disp_im);
    waitKey(10);

    Mat _3dImage(disp_im.rows, disp_im.cols, CV_32FC3);
    reprojectImageTo3D(disp_im, _3dImage, Q_mat, true);

    PointCloud::Ptr PointCloud_msg (new PointCloud);
    PointCloud_msg->header.frame_id = "map";
    PointCloud_msg->height = _3dImage.rows;
    PointCloud_msg->width  = _3dImage.cols;
    float point_x, point_y, point_z;

    //Mat_<double> vec_tmp(4,1);

    for(int u = 0; u < _3dImage.rows; u++)
        for(int v = 0; v < _3dImage.cols; v++)
        {

           
            point_x = _3dImage.at<Vec3b>(u,v)[0];
            point_y = _3dImage.at<Vec3b>(u,v)[1];
            point_z = _3dImage.at<Vec3b>(u,v)[2];
            if(point_z > 1000) continue;
            //cout << "u " << u << " v " << v << " " << point_x/100 << " " << point_y/100 << " " << point_z/100 << endl;
            PointCloud_msg->points.push_back(pcl::PointXYZ(point_x/100, point_y/100, point_z/100));
            //PointCloud_msg->points.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
        }
    //PointCloud_msg->height = PointCloud_msg->width = 1;
    //PointCloud_msg->header.stamp = ros::Time::now().toNSec();
    PointCloud_pub.publish(PointCloud_msg);

}  

*/


int main(int argc, char** argv)
{
    ros::init(argc, argv, "disp_to_pointcloud_node");
    ros::NodeHandle nh;
    PointCloud_pub = nh.advertise<PointCloud> ("points2", 1);
    //image_transport::ImageTransport it(nh);
    //image_transport::Subscriber disp_sub = it.subscribe("/sgbm_ros_node/disparity_image", 1, image_pointcloud_convert);

    depth_sub = nh.subscribe( "/sgbm_ros_node/depth_image",  1, rcvDepthCallback );
    cout << "Node inited. Check out rostopic!" << endl;
    ros::spin();
    return 0;
}

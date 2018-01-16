#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
// Converting ROS image messages to OpenCV images
#include <cv_bridge/cv_bridge.h>


class SegnetMap
{
	public:
		
	        SegnetMap(ros::NodeHandle nh);

		void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
	        void image_cb(const sensor_msgs::ImageConstPtr& msg);
		

	private:

		ros::Subscriber segnet_output;
		ros::Subscriber sub_point;
		ros::Publisher pub;

                  // マップ座標のtfのリンク名
                std::string map_frame_ /* = "map"*/;
                tf::TransformListener tf_listener_;
                 // cloud_cbで受け取ったポイントクラウドを保存する変数
		pcl::PointCloud<pcl::PointXYZ> g_callback_cloud;
		std::vector<sensor_msgs::PointCloud2> cloud_buffer;
                pcl::PointCloud<pcl::PointXYZRGB> color_point_map;
};


SegnetMap::SegnetMap(ros::NodeHandle nh)
{
        //sub_point     = nh.subscribe<sensor_msgs::PointCloud2>("/zed/point_cloud/cloud_registered",1000,&SegnetMap::cloud_cb, this);	
	sub_point     = nh.subscribe<sensor_msgs::PointCloud2>("/hokuyo3d/hokuyo_cloud2",1,&SegnetMap::cloud_cb, this);
	segnet_output = nh.subscribe<sensor_msgs::Image>("/segnet_output",1,&SegnetMap::image_cb, this);
	pub           = nh.advertise<sensor_msgs::PointCloud2>("/color_map", 1);

	map_frame_ = "map";

	ros::spin();
}


void SegnetMap::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
    sensor_msgs::PointCloud2 transformed_cloud;

    // センサ座標系からマップ座標系への変換
    tf::StampedTransform transform;
    try{
	    tf_listener_.waitForTransform(map_frame_, input->header.frame_id, ros::Time(),ros::Duration(100.0));
    }catch(tf::TransformException ex){
	    ROS_ERROR("%s",ex.what());
    }

    if(!pcl_ros::transformPointCloud(map_frame_, *input, 	transformed_cloud, tf_listener_)){
	    return;
    }

    cloud_buffer.push_back(transformed_cloud);
    
    
    //g_callback_cloud = *conv_input;
}

void  SegnetMap::image_cb(const sensor_msgs::ImageConstPtr& msg){
    sensor_msgs::PointCloud2 now_cloud;
    for(auto& i:cloud_buffer){
	    if((msg->header.stamp.toSec() - i.header.stamp.toSec()) < 0){
		    now_cloud = i;
		    break;
	    }
    }


    // ROSのPointCloudの型からpclのPointCloudに変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr conv_input(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(now_cloud, *conv_input);

    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat &mat = in_msg->image;
    for(int i=0; i<in_msg->image.cols; i++){
        for(int j=0; j<in_msg->image.rows; j++){
            int cell = i * in_msg->image.rows + j;
            
            pcl::PointXYZRGB color_point;
	    color_point.x = g_callback_cloud.points[cell].x;
	    color_point.y = g_callback_cloud.points[cell].y;
	    color_point.r = mat.at<cv::Vec3b>(i,j)[2];
	    color_point.g = mat.at<cv::Vec3b>(i,j)[1];
	    color_point.b = mat.at<cv::Vec3b>(i,j)[0];
            color_point_map.points.push_back(color_point);
            color_point_map.points[cell].z = 0.0;
        }
    }

         sensor_msgs::PointCloud2 output_cloud;
            // pcl_cloudは変換したいpcl::PointCloud
         pcl::toROSMsg(color_point_map, output_cloud);
     
          // inputはsubscribeしているsensor_msgs::PointCloud2型の変数
         //header (タイムスタンプとかが入っている) をコピーする
         output_cloud.header = msg->header;
        // frame_idを変更する．今回の場合はmap
         output_cloud.header.frame_id = "map";
         pub.publish(output_cloud);
}


int main(int argc ,char **argv)
{
	ros::init(argc, argv, "ros_segnet");
	ros::NodeHandle nh;
        SegnetMap sm(nh);	
        
	return 0;
}

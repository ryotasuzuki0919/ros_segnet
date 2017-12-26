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
#include <cv_bridge/cv_bridge.h>


class SegnetMap
{
	public:
		
	        SegnetMap();

		void cloud_cb(const sensor_msgs::PointCloud2Ptr& input);
	

	private:

		ros::Subscriber scan_sub;
		ros::Subscriber point_cloud_sub;
		ros::Subscriber segnet_output_sub;

                  // マップ座標のtfのリンク名
                std::string map_frame_ /* = "map"*/;
                tf::TransformListener tf_listener_;
                 // cloud_cbで受け取ったポイントクラウドを保存する変数
		pcl::PointCloud<pcl::PointXYZRGB> g_callback_cloud;
};



void SegnetMap::cloud_cb(const sensor_msgs::PointCloud2Ptr& input){
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
    
    // ROSのPointCloudの型からpclのPointCloudに変換
    pcl::PointCloud<pcl::PointXYZI>::Ptr conv_input(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(transformed_cloud, *conv_input);
    
    g_callback_cloud = *conv_input;
}

pcl::PointCloud<pcl::PointXYZRGB> color_point_map;
void image_cb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    for(int i=0; i<in_msg.image.cols; i++){
        for(int j=0; j<in_msg.image.rows; j++){
            int cell = i * in_msg.image_rows + j;
            color_point_map.push_back(g_callback_cloud.points[cell])
            color_point_map.points[cell].z = 0.0;
        }
    }
}


int main(int argc ,char **argv)
{
	ros::init(argc, argv, "ros_segnet");


	return 0;
}

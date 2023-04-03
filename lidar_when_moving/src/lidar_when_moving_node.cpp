#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

//This nodes takes a PointCloud2 as an input, and re-publishes it only if the robot moves (infered from ODOM tf (works both in real world and simu))
class LidarWhenMoving {
    protected:
        ros::Subscriber pc_sub_;
        ros::Subscriber odom_sub_;
        ros::Publisher filtered_pc_pub_;
        ros::NodeHandle nh_;
        tf::TransformListener listener_;
        double dist_thres_;
        double ang_thres_;
        bool is_moving_ = false;
        std::string robot_frame_;
        std::string odom_frame_;
        tf::Transform last_odom_pose_;

    protected:

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            tf::StampedTransform transform;
            try { 
                listener_.lookupTransform(odom_frame_,robot_frame_,ros::Time(0), transform);
            } catch (tf::TransformException &ex){
                ROS_ERROR("%s",ex.what());
                return;
            }
            if (isMoving(transform)) {
                pcl::PointCloud<pcl::PointXYZ> cloud;
                pcl::fromROSMsg(*msg, cloud);
                sensor_msgs::PointCloud2 output = *msg;
                output.header.stamp = ros::Time::now();
                filtered_pc_pub_.publish(output);
                last_odom_pose_ = transform;
            }
        }
        //void odom_callback(const nav_msgs::OdometryConstPtr msg) {
        //    if ((msg->twist.twist.linear.x > min_lin_vel_) || (msg->twist.twist.angular.z > min_ang_vel_)) {
        //        is_moving_ = true;
        //    } else {
        //        is_moving_ = false;
        //    }
        //}

        bool isMoving(const tf::StampedTransform& current) const {
            double angle_diff = static_cast<double>(current.getRotation().angleShortestPath(last_odom_pose_.getRotation()));
            if (angle_diff > ang_thres_) {
                return true;
            }
            double distance_diff = (current.getOrigin() - last_odom_pose_.getOrigin()).length();
            if (distance_diff > dist_thres_) {
                return true;
            }
            return false;
        }

    public:
        LidarWhenMoving() : nh_("~") {

            nh_.param<double>("dist_thres_",dist_thres_,0.005);//below this distance threshold, position is considered the same
            nh_.param<double>("ang_thres_",ang_thres_,0.0043);//below this angular threshold, orientation is considered the same (default 0.5degrees)
            nh_.param<std::string>("robot_frame",robot_frame_,"base_link");
            nh_.param<std::string>("odom_frame",odom_frame_,"odom");
            pc_sub_ = nh_.subscribe("input_scan",1,&LidarWhenMoving::pc_callback,this);
            filtered_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud",1);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"lidar_when_moving");
    LidarWhenMoving lwm;
    ros::spin();
}


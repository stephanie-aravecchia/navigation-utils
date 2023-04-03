#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

//This class takes a PointCloud2 as an input, and remove the points which are behind the lidar, in the robot width
class OusterFilter {
    protected:
        ros::Subscriber pc_sub_;
        ros::Publisher filtered_pc_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        double robot_width_;
        double x_offset_;
        double range_min_;

    protected:

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZI> cloud;
            pcl::PointCloud<pcl::PointXYZI> keep;
            pcl::fromROSMsg(*msg, cloud);
            size_t n = cloud.size();
            keep.reserve(n);
            for (auto it = cloud.points.begin(); it != cloud.points.end(); ++it){
                //if the point is behind the lidar, in the robot width, ignore
                if ((fabs(it->y) <= robot_width_/2) && (it->x < x_offset_)) {
                    continue;
                }
                //We ignore points below range_min
                if (sqrt(it->x * it->x + it->y * it->y + it->z * it->z) <= range_min_) {
                    continue;
                }
                //And finally, we ignore inf or nan
                if ((!std::isfinite(it->x)) || (!std::isfinite(it->y)) || (!std::isfinite(it->z))) {
                    continue;
                }
                keep.push_back(*it);
            }
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(keep, output);
            output.header.stamp = msg->header.stamp;
            output.header.frame_id = msg->header.frame_id;
            filtered_pc_pub_.publish(output);

        }

    public:
        OusterFilter() : nh_("~") {

            nh_.param<double>("robot_width",robot_width_,1.0);
            nh_.param<double>("x_offset",x_offset_,-.5);
            nh_.param<double>("range_min",range_min_,.5);
            pc_sub_ = nh_.subscribe("input_scan",1,&OusterFilter::pc_callback,this);
            filtered_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud",1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"ouster_filter");
    OusterFilter of;
    ros::spin();
}


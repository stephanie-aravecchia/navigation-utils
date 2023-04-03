#ifndef ADD_NOISE_H
#define ADD_NOISE_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <random>

/*This class takes a PointCloud2 as an input, and add some noise using a predefined model
The noise is applied on the Lidar range
It can be:
- Gaussian Noise with a fixed sigma 
- Gaussian Noise with sigma varying linearly with the range
- Gaussian noise with sigma varying quadratically with the range
- Random uniform between range_min and the actual range + delta
- A mixture of the above
*/
class NoisyPCL {
    protected:
        ros::Subscriber pc_sub_;
        ros::Publisher noisy_pc_pub_;

        ros::NodeHandle nh_;
        double sigma_;
        double alpha_;
        double range_min_;
        double range_max_;
        //mixture of noise ratios
        int mixture_normal_;
        int mixture_normal_alpha_r_;
        int mixture_normal_alpha_r2_;
        int mixture_uniform_;
        bool add_max_threshold_on_uniform_;
        bool add_max_threshold_on_alpha_;
        double delta_max_uniform_;
        enum class NoiseModel{Gaussian, Linear, Quadratic, Mixture, Uniform}; 
        NoiseModel noise_model_;

        std::default_random_engine engine_;
        
        struct SphericalPoint {
            float r;
            float theta;
            float phi;
            SphericalPoint() {}
            SphericalPoint(float r, float theta, float phi):r(r), theta(theta), phi(phi) {}
        };
    public:
        NoisyPCL();

    protected:
        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg);

        //Returns the noise value, drawn from a normal distribution with a constant sigma
        double getNoiseConstantSigma(double r);
        
        //Returns the noise value, drawn from a uniform distribution 
        //to set the noisy range between range_min and range + delta_unif
        double getNoiseUniform(double r);
        
        //Returns the noise value, drawn from a normal distribution with sigma linear function of the range 
        //(sigma = alpha * r) 
        double getNoiseLinearDistSigma(double r);
        
        //Returns the noise value, drawn from a a normal distribution with sigma quadratic function of the range
        //(sigma = alpha * r^2
        double getNoiseQuadraticDistSigma(double r);

        //Returns the noise value, drawn from a mixture of above distrubutions,
        //The mixture ratio is defined by the params
        double getNoiseMixtureSigma(double r);
        
        //utility function to change the system coordinate
        SphericalPoint cart2spherical(float x, float y, float z) const;
        //utility function to change the system coordinate
        pcl::PointXYZ spherical2cart(const SphericalPoint& p) const;

};
#endif
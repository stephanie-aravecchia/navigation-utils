#include "noisy_sensor/add_noise.h"
#include <math.h>

NoisyPCL::NoisyPCL() : nh_("~") {
        
    std::string noise;
    double normal_ratio;
    double normal_alpha_r_ratio;
    double normal_alpha_r2_ratio;
    double uniform_ratio;
    nh_.param<std::string>("noise_model",noise,"gaussian");
    nh_.param<double>("stddev",sigma_,.008);
    nh_.param<double>("alpha",alpha_,.01);
    nh_.param<double>("sensor_min_range",range_min_, .5);
    nh_.param<double>("sensor_max_range",range_max_, 50);
    //params below are in %, must sum to 1
    nh_.param<double>("mixture_normal_ratio",normal_ratio,0.88);
    nh_.param<double>("mixture_normal_alpha_r_ratio",normal_alpha_r_ratio,0.1);
    nh_.param<double>("mixture_normal_alpha_r2_ratio",normal_alpha_r2_ratio,0);
    nh_.param<double>("mixture_uniform_ratio",uniform_ratio,0.02);
    nh_.param<double>("delta_unif_",delta_max_uniform_, 0.03);//when random noise is added, it is drawn in (range_min, current_range+delta_max_uniform)
    nh_.param<bool>("add_max_threshold_noise_uniform",add_max_threshold_on_uniform_, true);//if set to false, uniform is drawn from (range_min, range_max) instead
    nh_.param<bool>("add_max_threshold_noise_alpha",add_max_threshold_on_alpha_, true);//if true, noisy points are always closer than real point
    if (fabs(normal_alpha_r2_ratio + normal_alpha_r_ratio + normal_ratio + uniform_ratio - 1)>=1e-6) {
        ROS_ERROR("Error in the choice of ratios in the mixture of noise. Must sum to 1.");
        exit(EXIT_FAILURE);
    }
    mixture_normal_ = static_cast<int>(normal_ratio * 1000);
    mixture_normal_alpha_r_ = static_cast<int>(normal_alpha_r_ratio * 1000);
    mixture_normal_alpha_r2_ = static_cast<int>(normal_alpha_r2_ratio * 1000);
    mixture_uniform_ = 1000 - mixture_normal_ - normal_alpha_r2_ratio - normal_alpha_r_ratio;
    assert(mixture_normal_>=0);
    pc_sub_ = nh_.subscribe("input_scan",1,&NoisyPCL::pc_callback,this);
    noisy_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("noisy_point_cloud",1);

    if (noise == "gaussian") {
        noise_model_ = NoiseModel::Gaussian; 
    } else if (noise == "linear_dist") {
        noise_model_ = NoiseModel::Linear;
    } else if (noise == "quadratic_dist") {
        noise_model_ = NoiseModel::Quadratic;
    } else if (noise == "mixture") {
        noise_model_ = NoiseModel::Mixture;
    } else if (noise == "mixture") {
        noise_model_ = NoiseModel::Uniform;
    } else {
        ROS_ERROR("Noise model does not exist.");
        exit(EXIT_FAILURE);
    }
}

void NoisyPCL::pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    unsigned int n = cloud.points.size();
    pcl::PointCloud<pcl::PointXYZ> tmp;
    tmp.reserve(n);
    //for each point, we convert the coordinates from cart 2 spherical, we add some noise to the range,
    //we convert back to cart and then we publish the message
    // points without return (r=0) are not dealt with correctly in the navigation stack, we dont republish them
    size_t kept = 0;
    for (auto it = cloud.points.begin(); it != cloud.points.end(); ++it){
        //First, we ignore (0,0,0) points 
        if (sqrt(it->x * it->x + it->y * it->y + it->z * it->z) <= range_min_) {
            continue;
        }
        SphericalPoint sphericalPoint = cart2spherical(it->x, it->y, it->z);
        switch (noise_model_) {
            case NoiseModel::Gaussian: 
                sphericalPoint.r += getNoiseConstantSigma(sphericalPoint.r);
                break;
            case NoiseModel::Linear: 
                sphericalPoint.r += getNoiseLinearDistSigma(sphericalPoint.r);
                break;
            case NoiseModel::Quadratic:
                sphericalPoint.r += getNoiseQuadraticDistSigma(sphericalPoint.r);
                break;
            case NoiseModel::Uniform:
                sphericalPoint.r += getNoiseUniform(sphericalPoint.r);
                break;
            case NoiseModel::Mixture:
                sphericalPoint.r += getNoiseMixtureSigma(sphericalPoint.r);
                break;
            //Default, no noise
            default:
                break;
        }
        pcl::PointXYZ noisyPoint = spherical2cart(sphericalPoint);
        //Finally, we ignore points in a cylinder of range_min above or below the robot, erroneous points
        if (hypot(noisyPoint.x, noisyPoint.y) < range_min_) {
            continue;
        }
        tmp.push_back(noisyPoint);
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(tmp, output);
    output.header.frame_id = msg->header.frame_id;
    noisy_pc_pub_.publish(output);

}

//The noise is a mixture of the following:
//gaussian noise with fixed sigma (i.e 90%)
//gaussian noise with sigma = alpha * r (i.e 8%)
//random  between range_min and r + max_random_thres(2%).
double NoisyPCL::getNoiseMixtureSigma(double r) {
    double noise;
    //ratio of different noise is defined by the params
    std::uniform_int_distribution<int> distri(1, 1000);
    int m = distri(engine_);
    if (m <= mixture_normal_) {
        return getNoiseConstantSigma(r);
    } else if (m <= (mixture_normal_ + mixture_normal_alpha_r_)) {
        return getNoiseLinearDistSigma(r);
    } else if (m <= (mixture_normal_ + mixture_normal_alpha_r_ + mixture_normal_alpha_r2_  )){
        return getNoiseQuadraticDistSigma(r);
    } else {
        return getNoiseUniform(r);
    }
}

double NoisyPCL::getNoiseUniform(double r) {
    if (add_max_threshold_on_uniform_) {
        std::uniform_real_distribution<double> distribution(range_min_, r + delta_max_uniform_);
        return distribution(engine_) - r;
    } else {
        std::uniform_real_distribution<double> distribution(range_min_, range_max_);
        return distribution(engine_) - r;
    }
}

double NoisyPCL::getNoiseConstantSigma(double r) {
    std::normal_distribution<double> distribution(0.0, sigma_);
    return distribution(engine_);
}

double NoisyPCL::getNoiseLinearDistSigma(double r) {
    double sigma = alpha_ * r;
    std::normal_distribution<double> distribution(0.0, sigma);
    double noise = distribution(engine_);
    if (add_max_threshold_on_alpha_) {
        return -fabs(noise);
    } else {
        return noise;

    }
}

double NoisyPCL::getNoiseQuadraticDistSigma(double r) {
    double sigma = alpha_ * r * r;
    std::normal_distribution<double> distribution(0.0, sigma);
    double noise = distribution(engine_);
    if (add_max_threshold_on_alpha_) {
        return -fabs(noise);
    } else {
        return noise;

    }
}

NoisyPCL::SphericalPoint NoisyPCL::cart2spherical(float x, float y, float z) const {
    return SphericalPoint(std::sqrt(x*x + y*y + z*z),std::atan2(std::hypot(x,y),z),std::atan2(y,x)); 
}

pcl::PointXYZ NoisyPCL::spherical2cart(const NoisyPCL::SphericalPoint& p) const {
    return pcl::PointXYZ(p.r*sin(p.theta)*cos(p.phi), p.r*sin(p.theta)*sin(p.phi), p.r*cos(p.theta));
}


int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"noisy_sensor");
    NoisyPCL pcl;
    ros::spin();
}
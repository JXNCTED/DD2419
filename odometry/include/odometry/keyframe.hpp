#include "pcl/point_cloud.h"

template <typename PointT>
class Keyframe
{
   public:
    Keyframe(pcl::PointCloud<PointT> cloud, Eigen::Matrix4d pose)
        : cloud(cloud), pose(pose)
    {
    }

    const pcl::PointCloud<PointT> &getCloud() const { return cloud; }

    const Eigen::Matrix4d &getPose() const { return pose; }

   private:
    pcl::PointCloud<PointT> cloud;
    Eigen::Matrix4d pose;
};
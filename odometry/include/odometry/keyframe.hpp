#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

template <typename PointT>
class Keyframe
{
   public:
    Keyframe(const typename pcl::PointCloud<PointT>::Ptr &cloud,
             const Eigen::Matrix4d &pose)
        : cloud(cloud), pose(pose)
    {
    }

    const typename pcl::PointCloud<PointT>::Ptr &getCloud() const
    {
        return cloud;
    }

    const Eigen::Matrix4d &getPose() const { return pose; }

   private:
    typename pcl::PointCloud<PointT>::Ptr cloud;
    Eigen::Matrix4d pose;
};
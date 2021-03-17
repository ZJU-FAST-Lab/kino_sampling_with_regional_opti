#ifndef _OCC_MAP_H
#define _OCC_MAP_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
// #include <tf2_ros/transform_listener.h>

#include <queue>

#define logit(x) (log((x) / (1 - (x))))
#define INVALID_IDX -1

using std::cout;
using std::endl;
using std::vector;
using std::pair;
using std::string;
using std::queue;
using std::shared_ptr;
using std::max;
using std::min;
using std::floor;
using std::ceil;
using std::isnan;
using std::isinf;

namespace tgk_planner
{
class OccMap
{
public:
  OccMap() {}
  ~OccMap() {};
  void init(const ros::NodeHandle& nh);

  bool odomValid() { return have_odom_; }
  bool mapValid() { return (global_map_valid_||local_map_valid_); }
  Eigen::Vector3d get_curr_posi() { return curr_posi_; }
	Eigen::Vector3d get_curr_twist() {return curr_twist_; }
  Eigen::Vector3d get_curr_acc() {return curr_acc_; }
  Eigen::Quaterniond get_curr_quaternion() {return curr_q_; }
  double getResolution() { return resolution_; }
  Eigen::Vector3d getOrigin() { return origin_; }
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);
  void setOccupancy(const Eigen::Vector3d &pos);
  int getVoxelState(const Eigen::Vector3d &pos);
  int getVoxelState(const Eigen::Vector3i &id);
  double getDistance(const Eigen::Vector3d &pos);
  double getDistance(const Eigen::Vector3i &id);
  float nearestObs(const double &x, const double &y, const double &z, float &x_obs, float &y_obs, float &z_obs);
	ros::Time getLocalTime() { return latest_odom_time_; };

  Eigen::Vector3i posToIndex(const Eigen::Vector3d &pos);
  void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d& pos);
  void indexToPos(const int &x, const int &y, const int &z, Eigen::Vector3d &pos);
  bool isInMap(const Eigen::Vector3d &pos);
  bool isInMap(const Eigen::Vector3i &id);
  Eigen::Vector3i getMapSize();
  
  typedef shared_ptr<OccMap> Ptr;
  
private:
  std::vector<double> occupancy_buffer_;  // 0 is free, 1 is occupied

  // map property
  Eigen::Vector3d min_range_, max_range_;  // map range in pos
  Eigen::Vector3i grid_size_;              // map size in index
  int grid_size_y_multiply_z_;
  Eigen::Vector3d local_range_min_, local_range_max_;

  
  bool isInLocalMap(const Eigen::Vector3d &pos);
  bool isInLocalMap(const Eigen::Vector3i &id);
  int idxToAddress(const int &x_id, const int &y_id, const int &z_id);
  int idxToAddress(const Eigen::Vector3i &id);

  Eigen::Vector3d origin_, map_size_;
  double resolution_, resolution_inv_;
  Eigen::Matrix4d T_ic0_, T_ic1_, T_ic2_, T_ic3_;

  bool have_odom_;
	Eigen::Vector3d curr_posi_, curr_twist_, curr_acc_;
	Eigen::Quaterniond curr_q_;

  // ros
  ros::NodeHandle node_;
  ros::Subscriber indep_odom_sub_;
	ros::Subscriber global_cloud_sub_;
  ros::Timer global_occ_vis_timer_, local_occ_vis_timer_;
	
  // for vis
	ros::Time latest_odom_time_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr curr_view_cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr history_view_cloud_ptr_;
	ros::Publisher curr_view_cloud_pub_, hist_view_cloud_pub_; 
	ros::Publisher pose_vis_pub_, twist_vis_pub_, acc_vis_pub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> SyncPolicyImageOdom;
	typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
  SynchronizerImageOdom sync_image_odom_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
	void depthOdomCallback(const sensor_msgs::ImageConstPtr& disp_msg, 
                         const nav_msgs::OdometryConstPtr& odom, 
                         const Eigen::Matrix4d& T_ic, 
                         Eigen::Matrix4d& last_T_wc, 
                         cv::Mat& last_depth_image, 
                         const string& camera_name);
  cv::Mat depth_image_;
  cv::Mat last_depth0_image_;
  Eigen::Matrix4d last_T_wc0_;
  void projectDepthImage(const Eigen::Matrix3d& K, 
                         const Eigen::Matrix4d& T_wc, const cv::Mat& depth_image, 
                         Eigen::Matrix4d& last_T_wc, cv::Mat& last_depth_image, ros::Time r_s);
  void raycastProcess(const Eigen::Vector3d& t_wc);
  int setCacheOccupancy(const Eigen::Vector3d &pos, int occ);

  void indepOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void globalOccVisCallback(const ros::TimerEvent& e);
  void localOccVisCallback(const ros::TimerEvent& e);
	void globalCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  bool has_global_cloud_, has_first_depth_, use_global_map_;
	bool global_map_valid_, local_map_valid_;

  // map fusion 
  double fx_, fy_, cx_, cy_;
  int rows_, cols_;
  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt_;
  vector<int> cache_hit_, cache_all_;
  vector<int> cache_traverse_, cache_rayend_;
  int raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;
	int img_col_, img_row_;
  Eigen::Matrix3d K_depth_;
  Eigen::Vector3d sensor_range_;
  Eigen::Vector3i sensor_range_grid_cnt_;
  bool show_raw_depth_, show_filter_proj_depth_;
  bool fully_initialized_;

  /* projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  bool use_shift_filter_;
  double depth_scale_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_;
	double min_occupancy_log_;
  double min_ray_length_, max_ray_length_;
  
	/* origin pcl show */
	void pubPointCloudFromDepth(const std_msgs::Header& header, 
                              const cv::Mat& depth_img, 
                              const Eigen::Matrix3d& intrinsic_K, 
                              const string& camera_name);
	ros::Publisher origin_pcl_pub_;
  ros::Publisher projected_pc_pub_;

  /* range query */
  pcl::KdTreeFLANN<pcl::PointXYZ> pc_kdtree_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_;

  /* ESDF */
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
  void updateESDF3d(const Eigen::Vector3i &min_esdf, const Eigen::Vector3i &max_esdf);
  std::vector<double> tmp_buffer1_, tmp_buffer2_, distance_buffer_;
  
public:
  void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2],
                            Eigen::Vector3d& diff) 
  {
    if (!isInMap(pos)) {
      // cout << "pos invalid for interpolation." << endl;
    }

    /* interpolation position */
    Eigen::Vector3d pos_m = pos - 0.5 * resolution_ * Eigen::Vector3d::Ones();
    Eigen::Vector3i idx;
    Eigen::Vector3d idx_pos;

    posToIndex(pos_m, idx);
    indexToPos(idx, idx_pos);
    diff = (pos - idx_pos) * resolution_inv_;

    for (int x = 0; x < 2; x++) {
      for (int y = 0; y < 2; y++) {
        for (int z = 0; z < 2; z++) {
          Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
          Eigen::Vector3d current_pos;
          indexToPos(current_idx, current_pos);
          pts[x][y][z] = current_pos;
        }
      }
    }
  }
  
};


inline int OccMap::idxToAddress(const int &x_id, const int &y_id, const int &z_id)
{
  return x_id * grid_size_y_multiply_z_ + y_id * grid_size_(2) + z_id;
}

inline int OccMap::idxToAddress(const Eigen::Vector3i &id)
{
  return id(0) * grid_size_y_multiply_z_ + id(1) * grid_size_(2) + id(2);
}

// takes about 0.04 us
inline int OccMap::getVoxelState(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return -1;
  if (!isInLocalMap(id))
    return 0;
  // (x, y, z) -> x*ny*nz + y*nz + z
  return occupancy_buffer_[idxToAddress(id)] > min_occupancy_log_ ? 1 : 0;
}

inline int OccMap::getVoxelState(const Eigen::Vector3i &id)
{
  if (!isInMap(id))
    return -1;
  if (!isInLocalMap(id))
    return 0;

  // (x, y, z) -> x*ny*nz + y*nz + z
  return occupancy_buffer_[idxToAddress(id)] > min_occupancy_log_ ? 1 : 0;
}

inline double OccMap::getDistance(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id)) return -1; // TODO -1 means out of map
  return distance_buffer_[idxToAddress(id)];
}

inline double OccMap::getDistance(const Eigen::Vector3i &id)
{
  return distance_buffer_[idxToAddress(id)];
}

inline bool OccMap::isInLocalMap(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idx;
  posToIndex(pos, idx);
  return isInLocalMap(idx);
}

inline bool OccMap::isInLocalMap(const Eigen::Vector3i &id)
{
  // Eigen::Vector3i min_id, max_id;
  // posToIndex(local_range_min_, min_id);
  // posToIndex(local_range_max_, max_id);
  // min_id(0) = max(0, min_id(0));
  // min_id(1) = max(0, min_id(1));
  // min_id(2) = max(0, min_id(2));
  // max_id(0) = min(grid_size_[0], max_id(0));
  // max_id(1) = min(grid_size_[1], max_id(1));
  // max_id(2) = min(grid_size_[2], max_id(2));
  // return (((id[0] - min_id[0]) | (max_id[0] - id[0]) | (id[1] - min_id[1]) | (max_id[1] - id[1]) | (id[2] - min_id[2]) | (max_id[2] - id[2])) >= 0);

  Eigen::Vector3i curr_id;
  posToIndex(curr_posi_, curr_id);
  Eigen::Vector3i dif_id = id - curr_id;
  if (dif_id[0] > sensor_range_grid_cnt_[0] || dif_id[0] < -sensor_range_grid_cnt_[0])
    return false;
  if (dif_id[1] > sensor_range_grid_cnt_[1] || dif_id[1] < -sensor_range_grid_cnt_[1])
    return false;
  if (dif_id[2] > sensor_range_grid_cnt_[2] || dif_id[2] < -sensor_range_grid_cnt_[2])
    return false;
  return true;
};

inline bool OccMap::isInMap(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idx;
  posToIndex(pos, idx);
  return isInMap(idx);
}

inline bool OccMap::isInMap(const Eigen::Vector3i &id)
{
  return ((id[0] | (grid_size_[0] - 1 - id[0]) | id[1] | (grid_size_[1] - 1 - id[1]) | id[2]| (grid_size_[2] - 1 - id[2])) >= 0);
};

inline void OccMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  id(0) = floor((pos(0) - origin_(0)) * resolution_inv_);
  id(1) = floor((pos(1) - origin_(1)) * resolution_inv_);
  id(2) = floor((pos(2) - origin_(2)) * resolution_inv_);
}

inline Eigen::Vector3i OccMap::posToIndex(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i id;
  id(0) = floor((pos(0) - origin_(0)) * resolution_inv_);
  id(1) = floor((pos(1) - origin_(1)) * resolution_inv_);
  id(2) = floor((pos(2) - origin_(2)) * resolution_inv_);
  return id;
}

inline void OccMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  pos = origin_;
  pos(0) += (id(0) + 0.5) * resolution_;
  pos(1) += (id(1) + 0.5) * resolution_;
  pos(2) += (id(2) + 0.5) * resolution_;
}

inline void OccMap::indexToPos(const int &x, const int &y, const int &z, Eigen::Vector3d &pos)
{
  pos = origin_;
  pos(0) += (x + 0.5) * resolution_;
  pos(1) += (y + 0.5) * resolution_;
  pos(2) += (z + 0.5) * resolution_;
}

inline Eigen::Vector3i OccMap::getMapSize()
{
  return grid_size_; 
}

}  // namespace tgk_planner

#endif

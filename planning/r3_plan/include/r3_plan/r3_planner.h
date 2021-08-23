#ifndef R3PLANNER_H
#define R3PLANNER_H

#include "occ_grid/pos_checker.h"
#include "nanoflann.hpp"
#include <memory>

// #include <ompl/base/SpaceInformation.h>
// #include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <ompl/geometric/planners/rrt/InformedRRTstar.h>
// #include <ompl/geometric/planners/prm/LazyPRMstar.h>
// #include <ompl/geometric/planners/prm/PRMstar.h>
// #include <ompl/geometric/planners/informedtrees/AITstar.h>
// #include <ompl/geometric/planners/informedtrees/BITstar.h>
// #include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/base/DiscreteMotionValidator.h>
#include <ros/ros.h>

namespace kino_planner
{
  struct R3Node
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    R3Node(){g_value = DBL_MAX; f_value = DBL_MAX; parent = nullptr;};
    R3Node(const Eigen::Vector3d & p) : pos(p){g_value = DBL_MAX; f_value = DBL_MAX; parent = nullptr;};
    Eigen::Vector3d pos;
    size_t id;
    double f_value; //cost_to_come + cost_to_go
    double g_value; //cost_to_come
    R3Node* parent;
    std::vector<std::pair<size_t, double>> neighbour_idx_dist;
  };
  
  struct KDTreeGraph
  {
    std::vector<R3Node> vertices;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return vertices.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
      if (dim == 0) return vertices[idx].pos.x();
      else if (dim == 1) return vertices[idx].pos.y();
      else return vertices[idx].pos.z();
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

  };

  typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, KDTreeGraph > ,
    KDTreeGraph,
    3 /* dim */
    > my_kd_tree_t;

  class R3NodeComparator 
  {
  public:
    bool operator()(R3Node* node1, R3Node* node2) 
    {
      return node1->f_value > node2->f_value;
    }
  };  

  class R3Planner
  {
  public:
    R3Planner();
    R3Planner(const ros::NodeHandle &nh, PosChecker::Ptr mapPtr) : nh_(nh), pos_checker_ptr_(mapPtr)
    {
      nh_.param("r3/sampling_space_inflate", sampling_space_inflate_, -1.0);
      nh_.param("r3/ground_height", ground_height_, -1.0);
      nh_.param("r3/ceiling_height", ceiling_height_, -1.0);
      nh_.param("r3/search_time_per_meter", search_time_per_meter_, -1.0);
      nh_.param("r3/lattice_step", lattice_step_, -1.0);
      nh_.param("r3/use_ompl", use_ompl_, false);
      ROS_WARN_STREAM("[r3] param: sampling_space_inflate: " << sampling_space_inflate_);
      ROS_WARN_STREAM("[r3] param: ground_height: " << ground_height_);
      ROS_WARN_STREAM("[r3] param: ceiling_height: " << ceiling_height_);
      ROS_WARN_STREAM("[r3] param: search_time_per_meter: " << search_time_per_meter_);
      ROS_WARN_STREAM("[r3] param: lattice_step: " << lattice_step_);
      ROS_WARN_STREAM("[r3] param: use ompl: " << use_ompl_);
    }

    bool planOnce(const Eigen::Vector3d &s, const Eigen::Vector3d &g, std::vector<Eigen::Vector3d> &p, double &cost, double radius)
    {
      // if (use_ompl_)
        // return plan(s, g, p, cost, search_time_per_meter_);
      // else
        return planPRM(s, g, p, cost, radius);
    }

    size_t getGraph(vector<vector<Eigen::Vector3d>> &edges)
    {
      edges.clear();
      for (size_t i = 0; i < vertice_number_; ++i)
      {
        size_t nb_size = graph_.vertices[i].neighbour_idx_dist.size();
        for (size_t j = 0; j < nb_size; ++j)
        {
          size_t nb_idx = graph_.vertices[i].neighbour_idx_dist[j].first;
          vector<Eigen::Vector3d> edge;
          edge.push_back(graph_.vertices[i].pos);
          edge.push_back(graph_.vertices[nb_idx].pos);
          edges.push_back(edge);
        }
      }
      return vertice_number_;
    }

    std::vector<Eigen::Vector3d> simplifyRoute(const std::vector<Eigen::Vector3d> &route, double resolution) const;

  private:
    // nodehandle params
    ros::NodeHandle nh_;
    double sampling_space_inflate_, ground_height_, ceiling_height_;
    double search_time_per_meter_;

    // environment
    PosChecker::Ptr pos_checker_ptr_;

    // PRM variables
    double lattice_step_;
    size_t vertice_number_;
    KDTreeGraph graph_;
    size_t start_id_, goal_id_;

    // bool plan(const Eigen::Vector3d &s, const Eigen::Vector3d &g, std::vector<Eigen::Vector3d> &p, double &cost, double timePerUnit) const;
    bool planPRM(const Eigen::Vector3d &s, const Eigen::Vector3d &g, std::vector<Eigen::Vector3d> &p, double &cost, double radius);
    void constructPRMGraph(double radius);
    size_t sampling(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);
    bool searchShortestPath(std::vector<Eigen::Vector3d> &p, double &cost);
    double getEuclideanHeuristic(const Eigen::Vector3d &s, const Eigen::Vector3d &g) const;
    void postProcessPath(std::vector<Eigen::Vector3d> &path);

    bool use_ompl_;
  };
} // namespace kino_planner
#endif
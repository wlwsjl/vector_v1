#ifndef VECTOR_ASSISTED_TELEOP_ASSISTED_TELEOP_H_
#define VECTOR_ASSISTED_TELEOP_ASSISTED_TELEOP_H_
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <Eigen/Core>

namespace assisted_teleop {
  class AssistedTeleop {
    public:
      AssistedTeleop();
      ~AssistedTeleop();
    private:
      void velCB(const geometry_msgs::TwistConstPtr& vel);
      void controlLoop();

      tf::TransformListener tf_;
      costmap_2d::Costmap2DROS costmap_ros_;
      double controller_frequency_;
      base_local_planner::TrajectoryPlannerROS planner_;
      boost::mutex mutex_;
      geometry_msgs::Twist cmd_vel_;
      boost::thread* planning_thread_;
      double theta_range_;
      int num_th_samples_, num_x_samples_;
      ros::Publisher pub_;
      ros::Subscriber sub_;
      double collision_trans_speed_, collision_rot_speed_;
      bool diff_drive_;
  };
};
#endif

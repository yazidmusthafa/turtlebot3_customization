/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *  Contributor: Pham Cong Trang <phamcongtranghd@gmail.com>
 *  Contributor: Mitchell Sayer <mitchell4408@gmail.com>
 */

 #include <algorithm>
 #include <string>
 #include <memory>
 
 #include "nav2_core/exceptions.hpp"
 #include "nav2_util/node_utils.hpp"
 #include "nav2_pure_pursuit_controller/pure_pursuit_controller.hpp"
 #include "nav2_util/geometry_utils.hpp"
 
 using std::hypot;
 using std::min;
 using std::max;
 using std::abs;
 using nav2_util::declare_parameter_if_not_declared;
 using nav2_util::geometry_utils::euclidean_distance;
 
 namespace nav2_pure_pursuit_controller
 {
 
 template<typename Iter, typename Getter>
 Iter min_by(Iter begin, Iter end, Getter getCompareVal)
 {
   if (begin == end) {
     return end;
   }
   auto lowest = getCompareVal(*begin);
   Iter lowest_it = begin;
   for (Iter it = ++begin; it != end; ++it) {
     auto comp = getCompareVal(*it);
     if (comp < lowest) {
       lowest = comp;
       lowest_it = it;
     }
   }
   return lowest_it;
 }
 
 void PurePursuitController::configure(
   const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
   std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
   const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
 {
   node_ = parent;
 
   auto node = node_.lock();
 
   costmap_ros_ = costmap_ros;
   tf_ = tf;
   plugin_name_ = name;
   logger_ = node->get_logger();
   clock_ = node->get_clock();
 
   declare_parameter_if_not_declared(
     node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.2));
   declare_parameter_if_not_declared(
     node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4));
   declare_parameter_if_not_declared(
     node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
   declare_parameter_if_not_declared(
     node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
 
   node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
   node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
   node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
   double transform_tolerance;
   node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
   transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
 
   global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
 }
 
 void PurePursuitController::cleanup()
 {
   RCLCPP_INFO(logger_, "Cleaning up controller: %s", plugin_name_.c_str());
   global_pub_.reset();
 }
 
 void PurePursuitController::activate()
 {
   RCLCPP_INFO(logger_, "Activating controller: %s", plugin_name_.c_str());
   global_pub_->on_activate();
 }
 
 void PurePursuitController::deactivate()
 {
   RCLCPP_INFO(logger_, "Deactivating controller: %s", plugin_name_.c_str());
   global_pub_->on_deactivate();
 }
 
 void PurePursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
 {
   (void)speed_limit;
   (void)percentage;
 }
 
 geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;

  auto transformed_plan = transformGlobalPlan(pose);

  // Find current progress on plan
  size_t current_index = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < transformed_plan.poses.size(); ++i) {
    double dist = euclidean_distance(pose, transformed_plan.poses[i]);
    if (dist < min_dist) {
      min_dist = dist;
      current_index = i;
    }
  }


  // Spin at halfway point for 3 seconds
  if (halfway_computed_ && !has_spun_) {
    // Find current robot position in original plan
    size_t current_index = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < original_plan_.poses.size(); ++i) {
      double dist = euclidean_distance(pose, original_plan_.poses[i]);
      if (dist < min_dist) {
        min_dist = dist;
        current_index = i;
      }
    }
  
    double traveled = 0.0;
    for (size_t i = 1; i <= current_index && i < original_plan_.poses.size(); ++i) {
      traveled += euclidean_distance(original_plan_.poses[i-1], original_plan_.poses[i]);
    }
  
    RCLCPP_INFO(logger_, "Distance traveled: %.3f / %.3f", traveled, halfway_distance_);
  
    if (!is_spinning_ && traveled >= halfway_distance_) {
      RCLCPP_INFO(logger_, "Reached halfway point! Starting 3-second spin.");
      spin_start_time_ = clock_->now();
      is_spinning_ = true;
    }
  
    if (is_spinning_) {
      rclcpp::Duration spin_duration = clock_->now() - spin_start_time_;
      if (spin_duration.seconds() < 6.0) {
        geometry_msgs::msg::TwistStamped spin_cmd;
        spin_cmd.header.frame_id = pose.header.frame_id;
        spin_cmd.header.stamp = clock_->now();
        spin_cmd.twist.linear.x = 0.0;
        spin_cmd.twist.angular.z = max_angular_vel_;
        RCLCPP_INFO(logger_, "Spinning... %.2f/3.00 seconds", spin_duration.seconds());
        return spin_cmd;
      } else {
        RCLCPP_INFO(logger_, "Spin completed.");
        has_spun_ = true;
        is_spinning_ = false;
      }
    }
  }
  
  

  RCLCPP_DEBUG(logger_, "Current index in transformed plan: %zu", current_index);
  RCLCPP_DEBUG(logger_, "Remaining poses in transformed plan: %zu", transformed_plan.poses.size() - current_index);

  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist_;
    });

  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }
  auto goal_pose = goal_pose_it->pose;

  double linear_vel, angular_vel;

  if (goal_pose.position.x > 0) {
    auto curvature = 2.0 * goal_pose.position.y /
      (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
    linear_vel = desired_linear_vel_;
    angular_vel = desired_linear_vel_ * curvature;
  } else {
    linear_vel = 0.0;
    angular_vel = max_angular_vel_;
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = max(-1.0 * abs(max_angular_vel_), min(angular_vel, abs(max_angular_vel_)));

  return cmd_vel;
}

 
void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_pub_->publish(path);
  global_plan_ = path;

  if (path.poses.empty()) {
    RCLCPP_WARN(logger_, "Received empty path.");
    return;
  }

  const auto & new_goal_pose = path.poses.back();

  bool goal_changed = !goal_pose_initialized_ ||
    euclidean_distance(new_goal_pose, last_goal_pose_) > 0.1;

  if (goal_changed) {
    RCLCPP_INFO(logger_, "New goal detected. Recomputing halfway point.");
    last_goal_pose_ = new_goal_pose;
    goal_pose_initialized_ = true;
    halfway_computed_ = true;
    has_spun_ = false;
    is_spinning_ = false;

    original_plan_ = path;  // Store the full original path

    halfway_index_ = path.poses.size() / 2;

    halfway_distance_ = 0.0;
    for (size_t i = 1; i <= halfway_index_; ++i) {
      halfway_distance_ += euclidean_distance(path.poses[i-1], path.poses[i]);
    }

    RCLCPP_INFO(logger_, "Halfway index: %zu, Distance to halfway: %.3f", halfway_index_, halfway_distance_);
  } else {
    RCLCPP_INFO(logger_, "Goal unchanged. Retaining previous halfway point.");
  }
}

 
 nav_msgs::msg::Path PurePursuitController::transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose)
 {
   if (global_plan_.poses.empty()) {
     throw nav2_core::PlannerException("Received plan with zero length");
   }
 
   geometry_msgs::msg::PoseStamped robot_pose;
   if (!transformPose(tf_, global_plan_.header.frame_id, pose, robot_pose, transform_tolerance_)) {
     throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
   }
 
   nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
   double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
     costmap->getResolution() / 2.0;
 
   auto transformation_begin = min_by(
     global_plan_.poses.begin(), global_plan_.poses.end(),
     [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
       return euclidean_distance(robot_pose, ps);
     });
 
   auto transformation_end = std::find_if(
     transformation_begin, end(global_plan_.poses),
     [&](const auto & global_plan_pose) {
       return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
     });
 
   auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
     geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
     stamped_pose.header.frame_id = global_plan_.header.frame_id;
     stamped_pose.header.stamp = pose.header.stamp;
     stamped_pose.pose = global_plan_pose.pose;
     transformPose(tf_, costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose, transform_tolerance_);
     return transformed_pose;
   };
 
   nav_msgs::msg::Path transformed_plan;
   std::transform(transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses), transformGlobalPoseToLocal);
   transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
   transformed_plan.header.stamp = pose.header.stamp;
 
   global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
   global_pub_->publish(transformed_plan);
 
   if (transformed_plan.poses.empty()) {
     throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
   }
 
   return transformed_plan;
 }
 
 bool PurePursuitController::transformPose(
   const std::shared_ptr<tf2_ros::Buffer> tf,
   const std::string frame,
   const geometry_msgs::msg::PoseStamped & in_pose,
   geometry_msgs::msg::PoseStamped & out_pose,
   const rclcpp::Duration & transform_tolerance) const
 {
   if (in_pose.header.frame_id == frame) {
     out_pose = in_pose;
     return true;
   }
 
   try {
     tf->transform(in_pose, out_pose, frame);
     return true;
   } catch (tf2::ExtrapolationException & ex) {
     auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
     if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) > transform_tolerance) {
       RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Transform data too old");
       return false;
     } else {
       tf2::doTransform(in_pose, out_pose, transform);
       return true;
     }
   } catch (tf2::TransformException & ex) {
     RCLCPP_ERROR(rclcpp::get_logger("tf_help"), "Exception in transformPose: %s", ex.what());
     return false;
   }
   return false;
 }
 
 }  // namespace nav2_pure_pursuit_controller
 
 PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)
 
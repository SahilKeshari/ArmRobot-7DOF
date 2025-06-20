#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

class CartesianPathFollower : public rclcpp::Node
{
public:
  CartesianPathFollower() : Node("multi_waypoint_cartesian"),
  move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), "arm_grp")
  {
    setup();
  }

private:

  // MoveIt interface
  moveit::planning_interface::MoveGroupInterface move_group;


  void setup()
  {
  try
    {

    using moveit::planning_interface::MoveGroupInterface;
    const moveit::core::RobotModelConstPtr &robot_model = move_group.getRobotModel();
    moveit::core::RobotState start_state(robot_model);

    std::vector<std::pair<std::string, double>> positions;
    positions = {{"lift_joint", 0}, {"joint1", 0},
                 {"joint2", 0}, {"joint3", 0},
                 {"joint4", 0}, {"joint5", 0},
                 {"joint6", 0}
                };

    for (const auto &joint : positions)
      {
          start_state.setJointPositions(joint.first, &joint.second);
          RCLCPP_INFO(this->get_logger(), "Set joint %s to position %f", joint.first.c_str(), joint.second);
      }

    RCLCPP_INFO(this->get_logger(), "Starting Cartesian waypoint execution...");

    geometry_msgs::msg::Pose start_pose = toPose(start_state.getGlobalLinkTransform(move_group.getEndEffectorLink()));
    move_group.setStartState(start_state);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose goal_pose;

    moveit::core::RobotState goal_state(start_state);

    std::vector<std::pair<std::string, double>> goal_positions;
    goal_positions = {{"lift_joint", 0.5}, {"joint1", 0},
                {"joint2", 0}, {"joint3", 0},
                {"joint4", 0}, {"joint5", 0},
                {"joint6", 0}
                };
    for (const auto &joint : goal_positions)
    {
        goal_state.setJointPositions(joint.first, &joint.second);
        RCLCPP_INFO(this->get_logger(), "Set joint %s to position %f", joint.first.c_str(), joint.second);
    }
    goal_state.update();
    goal_pose = toPose(goal_state.getGlobalLinkTransform(move_group.getEndEffectorLink()));
    
    waypoints.push_back(goal_pose);

    goal_positions = {
        {"lift_joint", 0.968406881739056},
        {"joint1", -1.2674782597856638},
        {"joint2", -1.225449832612562},
        {"joint3", -0.09166959098124657},
        {"joint4", 1.1337555806620478},
        {"joint5", 1.2673399189744319},
        {"joint6", -7.702271820526735e-05}
    };
    for (const auto &joint : goal_positions)
    {
        goal_state.setJointPositions(joint.first, &joint.second);
        RCLCPP_INFO(this->get_logger(), "Set joint %s to position %f", joint.first.c_str(), joint.second);
    }
    goal_state.update();
    goal_pose = toPose(goal_state.getGlobalLinkTransform(move_group.getEndEffectorLink()));
    
    waypoints.push_back(goal_pose);

    goal_positions = {{"lift_joint", 0}, {"joint1", 0},
                 {"joint2", 0}, {"joint3", 0},
                 {"joint4", 0}, {"joint5", 0},
                 {"joint6", 0}
                };
    for (const auto &joint : goal_positions)
    {
        goal_state.setJointPositions(joint.first, &joint.second);
        RCLCPP_INFO(this->get_logger(), "Set joint %s to position %f", joint.first.c_str(), joint.second);
    }
    goal_state.update();
    goal_pose = toPose(goal_state.getGlobalLinkTransform(move_group.getEndEffectorLink()));
    
    waypoints.push_back(goal_pose);

    for(int i=0; i<2; i++){
      for(int j=0; j<3; j++){
        waypoints.push_back(waypoints[j]);
      }
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;
    double fraction = move_group.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(this->get_logger(), "Path computed: %.2f%% achieved", fraction * 100.0);

    if (fraction > 0.95)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      move_group.execute(plan);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Path incomplete or failed.");
    }
  }
  catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error during planning: %s", e.what());
    }
}

  geometry_msgs::msg::Pose toPose(const Eigen::Isometry3d &transform)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = transform.translation().x();
        pose.position.y = transform.translation().y();
        pose.position.z = transform.translation().z();

        Eigen::Quaterniond quat(transform.rotation());
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();

        RCLCPP_DEBUG(this->get_logger(), "Converted Eigen::Isometry3d to geometry_msgs::msg::Pose:");
        RCLCPP_DEBUG(this->get_logger(), "  - Position: [%f, %f, %f]", pose.position.x, pose.position.y, pose.position.z);
        RCLCPP_DEBUG(this->get_logger(), "  - Orientation: [%f, %f, %f, %f]", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        return pose;
  }


};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianPathFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

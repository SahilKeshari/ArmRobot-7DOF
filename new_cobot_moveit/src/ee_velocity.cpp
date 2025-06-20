#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class EndEffectorVelocity : public rclcpp::Node
{
public:
  EndEffectorVelocity()
  : Node("ee_velocity_from_tf"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    xyz_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/xyz_velocity", 10);
    eef_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/eef_velocity", 10);
    eef_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/eef", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 100 Hz
      std::bind(&EndEffectorVelocity::timerCallback, this));

    prev_time_ = this->now();
    has_prev_ = false;
    prev_speed = 0.0;
    i=0;
  }

private:
  void timerCallback()
  {
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform("base_link", "link6", tf2::TimePointZero);
    }
    catch (tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    rclcpp::Time current_time = transform.header.stamp;
    double t = current_time.seconds();

    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;
    double z = transform.transform.translation.z;
    double speed = 0.0;
    std_msgs::msg::Float64MultiArray eef_array_msg;
    eef_array_msg.data = {x, y, z};
    eef_publisher_->publish(eef_array_msg);
    
    if (has_prev_)
    {
      double dt = (current_time - prev_time_).seconds();
      if (dt > 0)
      {

        double vx = (x - prev_pos_[0]) / dt ;
        double vy = (y - prev_pos_[1]) / dt ;
        double vz = (z - prev_pos_[2]) / dt ;

        speed = std::sqrt(vx * vx + vy * vy + vz * vz);  // Total velocity

        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header = transform.header;
        twist_msg.header.frame_id = "eef";

        twist_msg.twist.linear.x = vx;
        twist_msg.twist.linear.y = vy;
        twist_msg.twist.linear.z = vz;

        eef_vel_msg.data = speed;
        eef_vel_publisher_->publish(eef_vel_msg);

        xyz_vel_publisher_->publish(twist_msg);
      }
    }
    prev_speed = speed;
    prev_pos_[0] = x;
    prev_pos_[1] = y;
    prev_pos_[2] = z;
    prev_time_ = current_time;
    has_prev_ = true;
    i++;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr xyz_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr eef_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr eef_publisher_;
  std_msgs::msg::Float64 eef_vel_msg;
  // double prev_vx, prev_vy, prev_vz;

  double prev_pos_[3];
  double prev_vel_[3];

  rclcpp::Time prev_time_;
  bool has_prev_;
  double prev_speed;
  int i;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EndEffectorVelocity>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

class HalfwaySpinNode : public rclcpp::Node
{
public:
    HalfwaySpinNode() : Node("halfway_spin_node")
    {
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&HalfwaySpinNode::goalCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&HalfwaySpinNode::odomCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        lifecycle_client_ = create_client<lifecycle_msgs::srv::ChangeState>(
            "/controller_server/manage_lifecycle_node");
    }

private:
    geometry_msgs::msg::PoseStamped goal_;
    nav_msgs::msg::Odometry current_odom_;
    bool goal_received_ = false;
    bool halfway_spun_ = false;

    double initial_distance_ = 0.0;  // Track initial distance when goal is received

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr lifecycle_client_;

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_ = *msg;
        goal_received_ = true;
        halfway_spun_ = false;

        initial_distance_ = computeDistance(
            current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y,
            goal_.pose.position.x, goal_.pose.position.y
        );

        RCLCPP_INFO(get_logger(), "New goal received. Initial distance to goal: %.2f meters", initial_distance_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;

        if (goal_received_ && !halfway_spun_)
        {
            if (isHalfway())
            {
                RCLCPP_INFO(get_logger(), "Halfway reached - Pausing Nav2 and performing spin.");

                // Pause Nav2's controller
                manageControllerLifecycle(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

                // Perform spin
                performSpin();

                // Resume Nav2's controller
                manageControllerLifecycle(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

                RCLCPP_INFO(get_logger(), "Spin complete - Resuming Nav2.");
                halfway_spun_ = true;
            }
        }
    }

    bool isHalfway()
    {
        double current_x = current_odom_.pose.pose.position.x;
        double current_y = current_odom_.pose.pose.position.y;

        double remaining_distance = computeDistance(
            current_x, current_y,
            goal_.pose.position.x, goal_.pose.position.y
        );

        double halfway_distance = initial_distance_ / 2.0;
        return (remaining_distance <= halfway_distance);
    }

    double computeDistance(double x1, double y1, double x2, double y2)
    {
        return std::hypot(x2 - x1, y2 - y1);
    }

    void performSpin()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = 0.5;  // Rotate at 0.5 rad/s

        rclcpp::Time start_time = now();
        rclcpp::Rate rate(20);  // 20 Hz

        double spin_duration = 2 * M_PI / 0.5;  // Full 360 degrees rotation

        while ((now() - start_time).seconds() < spin_duration)
        {
            cmd_vel_pub_->publish(cmd);
            rate.sleep();
        }

        // Stop rotation
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
    }

    void manageControllerLifecycle(uint8_t transition_id)
    {
        if (!lifecycle_client_->wait_for_service(std::chrono::seconds(3)))
        {
            RCLCPP_ERROR(get_logger(), "Lifecycle service not available!");
            return;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition_id;

        auto future_result = lifecycle_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            const char* command_str = (transition_id == lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) ? "DEACTIVATED" : "ACTIVATED";
            RCLCPP_INFO(get_logger(), "Controller successfully %s.", command_str);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to manage controller lifecycle.");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HalfwaySpinNode>());
    rclcpp::shutdown();
    return 0;
}

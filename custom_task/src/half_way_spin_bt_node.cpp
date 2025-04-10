#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/spin.hpp>

class HalfwaySpinNode : public rclcpp::Node
{
public:
    using Spin = nav2_msgs::action::Spin;
    using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

    HalfwaySpinNode() : Node("halfway_spin_node")
    {
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&HalfwaySpinNode::goalCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&HalfwaySpinNode::odomCallback, this, std::placeholders::_1));

        spin_action_client_ = rclcpp_action::create_client<Spin>(this, "/spin");

        RCLCPP_INFO(get_logger(), "HalfwaySpinNode started.");
    }

private:
    geometry_msgs::msg::PoseStamped goal_;
    nav_msgs::msg::Odometry current_odom_;

    bool goal_received_ = false;
    bool halfway_spun_ = false;  // Ensures spin only happens once per goal
    bool spin_active_ = false;   // Tracks if a spin is currently in progress
    double initial_distance_ = 0.0;  // Distance to goal at goal reception

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Client<Spin>::SharedPtr spin_action_client_;

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_ = *msg;
        goal_received_ = true;
        halfway_spun_ = false;
        spin_active_ = false;

        initial_distance_ = computeDistance(
            current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y,
            goal_.pose.position.x, goal_.pose.position.y);

        RCLCPP_INFO(get_logger(), "Received new goal - Distance to goal: %.2f meters", initial_distance_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;

        if (goal_received_ && !halfway_spun_ && !spin_active_ && isHalfway())
        {
            RCLCPP_INFO(get_logger(), "Halfway to goal - triggering spin behavior.");
            triggerSpinAction();
        }
    }

    bool isHalfway()
    {
        double current_x = current_odom_.pose.pose.position.x;
        double current_y = current_odom_.pose.pose.position.y;

        double remaining_distance = computeDistance(
            current_x, current_y,
            goal_.pose.position.x, goal_.pose.position.y);

        double halfway_distance = initial_distance_ / 2.0;

        return (remaining_distance <= halfway_distance);
    }

    double computeDistance(double x1, double y1, double x2, double y2)
    {
        return std::hypot(x2 - x1, y2 - y1);
    }

    void triggerSpinAction()
    {
        if (!spin_action_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(get_logger(), "Spin action server not available!");
            return;
        }

        auto goal_msg = Spin::Goal();
        goal_msg.target_yaw = 2 * M_PI;  // 360-degree spin

        auto send_goal_options = rclcpp_action::Client<Spin>::SendGoalOptions();

        send_goal_options.goal_response_callback = std::bind(
            &HalfwaySpinNode::onSpinGoalResponse, this, std::placeholders::_1);

        send_goal_options.result_callback = std::bind(
            &HalfwaySpinNode::onSpinResult, this, std::placeholders::_1);

        spin_action_client_->async_send_goal(goal_msg, send_goal_options);
        spin_active_ = true;  // Lock out further spin triggers until complete
    }

    void onSpinGoalResponse(const GoalHandleSpin::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Spin action goal rejected.");
            spin_active_ = false;  // Unlock in case of failure
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Spin action goal accepted.");
        }
    }

    void onSpinResult(const GoalHandleSpin::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Spin completed successfully.");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Spin action aborted.");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Spin action canceled.");
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code from spin action.");
            break;
        }

        halfway_spun_ = true;  // Ensure we don't spin again on this goal
        spin_active_ = false;  // Allow future spins for new goals
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HalfwaySpinNode>());
    rclcpp::shutdown();
    return 0;
}

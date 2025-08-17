#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <vector>
#include <string>
#include <fstream>

namespace autonomous_navigation
{

enum class NavigationState
{
    IDLE,           // 待機中
    NAVIGATING,     // ナビゲーション中
    PAUSED,         // 一時停止（障害物）
    COMPLETED       // 全ウェイポイント完了
};

struct WayPoint
{
    double x, y, z;
    double qx, qy, qz, qw;  
};

class AutodriveManager : public rclcpp::Node
{
public:
    explicit AutodriveManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~AutodriveManager();

    // === パス管理 ===
    bool loadPathFromCSV(const std::string& csv_file_path);
    void startNavigation();

private:
    void setParam();
    // === コールバック関数 ===
    void timerCallback();
    void goalReachedCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void obstacleCallback(const std_msgs::msg::Bool::SharedPtr msg);
    
    // === 内部処理 ===
    void publishCurrentGoal();
    void publishPriorityVelocity();
    void publishNavigationState();
    void moveToNextWaypoint();
    
    // === CSV読み込み ===
    std::vector<std::string> splitCSVLine(const std::string& line);

    // === ROS2 インターフェース ===
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr priority_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // === 状態変数 ===
    std::vector<WayPoint> waypoints_;
    size_t current_waypoint_index_ = 0;
    size_t previous_waypoint_index_ = -1;
    NavigationState state_;

    bool goal_reached_ = false;
    bool navigation_active_ = false;

    double publish_frequency_;           
    std::string map_frame_;        
    std::string csv_path_;    
};

} // namespace autonomous_navigation

#endif // AUTONOMOUS_NAVIGATION_SIMPLE_NAVIGATOR_HPP

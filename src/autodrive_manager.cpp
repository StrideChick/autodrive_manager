#include "autodrive_manager/autodrive_manager.hpp"
#include <sstream>
#include <iostream>

namespace autonomous_navigation
{
AutodriveManager::AutodriveManager(const rclcpp::NodeOptions& options)
: Node("autodrive_manager", options)
  {
    setParam();
    if (loadPathFromCSV(csv_path_)) {
        startNavigation();
    }
    // publisher
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    priority_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("priority_velocity", 10);
    state_publisher_ = create_publisher<std_msgs::msg::String>("navigation_state", 10);

    // subscriber
    goal_reached_subscriber_ = create_subscription<std_msgs::msg::Bool>(
        "goal_reached", 10,
        std::bind(&AutodriveManager::goalReachedCallback, this, std::placeholders::_1));
    
    obstacle_subscriber_ = create_subscription<std_msgs::msg::Bool>(
        "obstacle_detected", 10,
        std::bind(&AutodriveManager::obstacleCallback, this, std::placeholders::_1));

    // timer
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_frequency_)),
        std::bind(&AutodriveManager::timerCallback, this));
}

AutodriveManager::~AutodriveManager()
{
    RCLCPP_INFO(get_logger(), "AutodriveManager destroyed");
}

void AutodriveManager::setParam()
{
    state_ = NavigationState::IDLE;
    // param setting
    declare_parameter("publish_frequency", 10.0);
    declare_parameter("map_frame", "map");
    declare_parameter("csv_path", "");
    publish_frequency_ = get_parameter("publish_frequency").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    csv_path_ = get_parameter("csv_path").as_string();
}


bool AutodriveManager::loadPathFromCSV(const std::string& csv_file_path)
{
    std::ifstream file(csv_file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_file_path.c_str());
        return false;
    }

    waypoints_.clear();
    std::string line;
    bool is_header = true;
    size_t line_number = 0;

    while (std::getline(file, line)) {
        line_number++;
        if (is_header) {
            is_header = false;
            continue; // ヘッダー行をスキップ
        }
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ',')) {
            // 前後の空白をトリム
            token.erase(0, token.find_first_not_of(" \t\r\n"));
            token.erase(token.find_last_not_of(" \t\r\n") + 1);
            tokens.push_back(token);
        }

        if (tokens.size() < 7) {
            RCLCPP_WARN(this->get_logger(),
                        "Line %zu skipped: expected 7 columns, got %zu", 
                        line_number, tokens.size());
            continue;
        }

        try {
            WayPoint wp;
            wp.x  = std::stod(tokens[0]);
            wp.y  = std::stod(tokens[1]);
            wp.z  = std::stod(tokens[2]);
            wp.qx = std::stod(tokens[3]);
            wp.qy = std::stod(tokens[4]);
            wp.qz = std::stod(tokens[5]);
            wp.qw = std::stod(tokens[6]);
            waypoints_.push_back(wp);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to parse line %zu: %s (%s)", 
                        line_number, line.c_str(), e.what());
        }
    }

    file.close();
    return !waypoints_.empty();
}

void AutodriveManager::startNavigation()
{
    if (waypoints_.empty()) {
        RCLCPP_WARN(get_logger(), "No waypoints loaded. Cannot start navigation.");
        return;
    }
    current_waypoint_index_ = 0;
    state_ = NavigationState::NAVIGATING;
    navigation_active_ = true;
    goal_reached_ = false;
}

void AutodriveManager::goalReachedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!navigation_active_ || state_ == NavigationState::PAUSED) return;
    if (msg->data && state_ == NavigationState::NAVIGATING) {
        goal_reached_ = true;
        moveToNextWaypoint();
    }
}


void AutodriveManager::obstacleCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool obstacle_detected = msg->data;
    if (obstacle_detected) {
        if (state_ == NavigationState::NAVIGATING) {
            state_ = NavigationState::PAUSED;
            RCLCPP_WARN(get_logger(), "Obstacle detected!");
        }
    }else{
        if (state_ == NavigationState::PAUSED) {
            state_ = NavigationState::NAVIGATING;
            RCLCPP_INFO(this->get_logger(), "Obstacle cleared. Navigation resumed.");
        }
    }
}

void AutodriveManager::timerCallback()
{
    if (!navigation_active_) return;
    switch (state_) {
        case NavigationState::NAVIGATING:
            if (previous_waypoint_index_ != current_waypoint_index_) {
                if (current_waypoint_index_ < waypoints_.size()) {
                    publishCurrentGoal();
                    previous_waypoint_index_ = current_waypoint_index_;
                }
            }
            break;
        case NavigationState::PAUSED:
            publishPriorityVelocity();
            break;
        case NavigationState::COMPLETED:
            break;
        default:
            break;
    }
    publishNavigationState();
}

void AutodriveManager::moveToNextWaypoint()
{
    current_waypoint_index_++;
    if (current_waypoint_index_ >= waypoints_.size()) {
        state_ = NavigationState::COMPLETED;
        navigation_active_ = false;
        goal_reached_ = false;
    } else {
        goal_reached_ = false;
    }
}

void AutodriveManager::publishCurrentGoal()
{
    if (current_waypoint_index_ >= waypoints_.size()) return;
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->get_clock()->now();
    goal_msg.header.frame_id = map_frame_;

    const WayPoint& wp = waypoints_[current_waypoint_index_];
    goal_msg.pose.position.x = wp.x;
    goal_msg.pose.position.y = wp.y;
    goal_msg.pose.position.z = wp.z;
    goal_msg.pose.orientation.x = wp.qx;
    goal_msg.pose.orientation.y = wp.qy;
    goal_msg.pose.orientation.z = wp.qz;
    goal_msg.pose.orientation.w = wp.qw;
    goal_publisher_->publish(goal_msg);
}

void AutodriveManager::publishPriorityVelocity()
{
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    priority_vel_publisher_->publish(vel_msg);
}

void AutodriveManager::publishNavigationState()
{
    std_msgs::msg::String state_msg;
    switch (state_) {
        case NavigationState::IDLE:
            state_msg.data = "IDLE";
            break;
        case NavigationState::NAVIGATING:
            state_msg.data = "NAVIGATING";
            break;
        case NavigationState::PAUSED:
            state_msg.data = "PAUSED";
            break;
        case NavigationState::COMPLETED:
            state_msg.data = "COMPLETED";
            break;
    }
    state_publisher_->publish(state_msg);
}
} // namespace autonomous_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto navigator = std::make_shared<autonomous_navigation::AutodriveManager>();
    rclcpp::spin(navigator);
    rclcpp::shutdown();    
    return 0;
}
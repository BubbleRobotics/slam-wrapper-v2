/* *************************************************************************** */
/*                                                    ########  ########       */
/*   stereo.cpp                                       ##     ## ##     ##      */
/*                                                    ##     ## ##     ##      */
/*   By: Diego Hernandez <diego@bubble-robotics.com>  ########  ########       */
/*                                                    ##     ## ##   ##        */
/*   Created: 2026/01/05 17:07:03 by Diego Hernandez  ##     ## ##    ##       */
/*   Updated: 2026/01/05 17:07:03 by Diego Hernandez  ########  ##     ##      */
/*                                                                             */
/* *************************************************************************** */

//* Includes
#include "ros2_orb_slam3/dvl.hpp"

//* Constructor
DVLMode::DVLMode() : Node("dvl_node")
{
  RCLCPP_INFO(this->get_logger(), "DVL EKF NODE STARTED");

  this->declare_parameter("dvl_topic", "/dvl/twist_data");
  this->declare_parameter("odometry_est", "/ORB_SLAM3/stereo_sim_node/odometry");
  this->declare_parameter("verbose", false);

  // Populate Variables
  rclcpp::Parameter dvlTopicParam = this->get_parameter("dvl_topic");
  dvlTopic = dvlTopicParam.as_string();
  rclcpp::Parameter odometryEstParam = this->get_parameter("odometry_est");
  odometryEst = odometryEstParam.as_string();
  rclcpp::Parameter verboseParam = this->get_parameter("verbose");
  verbose = verboseParam.as_bool();

  // DEBUG print
  RCLCPP_INFO(this->get_logger(), "DVL Topic: %s", dvlTopic.c_str());
  RCLCPP_INFO(this->get_logger(), "Odometry Estimation Topic: %s", odometryEst.c_str());

  // Creates Subscribers
  dvlSub_ = this->create_subscription<dvl_msgs::msg::DVL>(dvlTopic, rclcpp::SensorDataQoS(), std::bind(&DVLMode::DvlCallback, this, std::placeholders::_1));
  odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(odometryEst, rclcpp::SensorDataQoS(), std::bind(&DVLMode::OdomCallback, this, std::placeholders::_1));

  // Creates topic called /DVL_EKF/dvl_node/odometry
  odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10); 

  // Creates Client to Initialize the DVL messages
  init_client_ = this->create_client<std_srvs::srv::Trigger>("/initialize");
  init_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&DVLMode::InitializeService, this));
  initialized_dvl = false;
}

DVLMode::~DVLMode()
{
  RCLCPP_INFO(this->get_logger(), "DVL EKF NODE SHUTTING DOWN");
}

void DVLMode::DvlCallback(const dvl_msgs::msg::DVL::ConstSharedPtr &msg)
{
    if (initialized_dvl && verbose)
    {
        // we can use the covariance matrix and the validity bool to decide wether we put the data in the queue
        double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        RCLCPP_INFO(this->get_logger(), "DVL Data:");
        RCLCPP_INFO(this->get_logger(), "  Time: %.6f", t);
        RCLCPP_INFO(this->get_logger(), "  Velocity Valid: %s", msg->velocity_valid ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  Velocity [m/s]: [%.4f, %.4f, %.4f]",
                    msg->velocity.x,
                    msg->velocity.y,
                    msg->velocity.z);
        RCLCPP_INFO(this->get_logger(), "  Altitude [m]: %.3f",
                    msg->altitude);

        const auto & cov = msg->covariance;

        if (cov.size() == 9)
        {
            RCLCPP_INFO(this->get_logger(), "  Covariance matrix (m/s)^2:");
            RCLCPP_INFO(this->get_logger(), "    [%.6e %.6e %.6e]", cov[0], cov[1], cov[2]);
            RCLCPP_INFO(this->get_logger(), "    [%.6e %.6e %.6e]", cov[3], cov[4], cov[5]);
            RCLCPP_INFO(this->get_logger(), "    [%.6e %.6e %.6e]", cov[6], cov[7], cov[8]);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "  Covariance size is %zu (expected 9)", cov.size());
        }
    }
}

void DVLMode::OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
    if (initialized_dvl && verbose)
    {
        double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        RCLCPP_INFO(this->get_logger(), "Odometry Data:");
        RCLCPP_INFO(this->get_logger(), "  Time: %.6f", t);
        RCLCPP_INFO(this->get_logger(), "  Position [m]: [%.4f, %.4f, %.4f]",
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Orientation [rad]: [%.4f, %.4f, %.4f, %.4f]",
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
        const auto &cov = msg->pose.covariance;

        RCLCPP_INFO(this->get_logger(),
            "  Pose covariance diag: "
            "x=%.6f y=%.6f z=%.6f roll=%.6f pitch=%.6f yaw=%.6f",
            cov[0],    // x-x
            cov[7],    // y-y
            cov[14],   // z-z
            cov[21],   // roll-roll
            cov[28],   // pitch-pitch
            cov[35]    // yaw-yaw
        );
    }
}

void DVLMode::InitializeService()
{
    if (!init_client_->wait_for_service(std::chrono::seconds(0))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /initialize service...");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;

    auto response_callback = [this] (ServiceResponseFuture future) {
        auto response = future.get();

        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "DVL Initialization successful: %s", response->message.c_str());
            init_timer_->cancel(); // Stop the timer once initialized
            initialized_dvl = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "DVL Initialization failed: %s", response->message.c_str());
        }
    };
    init_client_->async_send_request(request, response_callback);
}
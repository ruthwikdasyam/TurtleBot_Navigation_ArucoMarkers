#pragma once

#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>  
#include "mage_msgs/msg/part_pose.hpp"
#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


/**
 * @brief Class - MySubscriber
 * Contians
 * 1) 7 subscribers for 5 camera topics, 1 /arucomarker and 1 /amcl_pose topic
 * 2) 2 publishers - 1 for intialpose of robot and 1 for waypoint publisher
 * 
 * This Class "MySubscriber" subscribes to multiple topics and stores necessary data and unsubscribes
 * The aruco subscriber stores the aruco id
 * The 5 camera subscribers gets the position of the parts and stores them
 * It then, transforms the position wrt "map" frame
 * 
 * Other methods contains action server that uses pulishWaypoints method to publish waypoints to the turtlebot
 * 
 * 
 */
class MySubscriber : public rclcpp::Node
{

public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    /**
     * @brief Constructor for MySubscriber Class
     * 
     * @param node_name - input node_name while creating an object
     */
    MySubscriber(std::string node_name): Node(node_name)
    {
        std::this_thread::sleep_for(std::chrono::seconds(5));
  
        this->declare_parameter("aruco_0.wp1.type","battery");
        this->declare_parameter("aruco_0.wp1.color","green");
        this->declare_parameter("aruco_0.wp2.type","battery");
        this->declare_parameter("aruco_0.wp2.color","red");
        this->declare_parameter("aruco_0.wp3.type","battery");
        this->declare_parameter("aruco_0.wp3.color","orange");
        this->declare_parameter("aruco_0.wp4.type","battery");
        this->declare_parameter("aruco_0.wp4.color","purple");
        this->declare_parameter("aruco_0.wp5.type","battery");
        this->declare_parameter("aruco_0.wp5.color","blue");

        this->declare_parameter("aruco_1.wp1.type","battery");
        this->declare_parameter("aruco_1.wp1.color","blue");
        this->declare_parameter("aruco_1.wp2.type","battery");
        this->declare_parameter("aruco_1.wp2.color","green");
        this->declare_parameter("aruco_1.wp3.type","battery");
        this->declare_parameter("aruco_1.wp3.color","orange");
        this->declare_parameter("aruco_1.wp4.type","battery");
        this->declare_parameter("aruco_1.wp4.color","red");
        this->declare_parameter("aruco_1.wp5.type","battery");
        this->declare_parameter("aruco_1.wp5.color","purple");

        // initialize the publisher
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

        // Create a publisher for geometry_msgs::msg::PoseStamped
        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("waypoints", 10);

        // Create an action client for nav2_msgs::action::NavigateThroughPoses
        action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");

        // Create a timer to publish waypoints periodically
        // timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&MySubscriber::publishWaypoints, this));

        // Create a subscriber for the "aruco" topic
        subscriber_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "aruco_markers", 10,
            std::bind(&MySubscriber::subscriber_callback, this, std::placeholders::_1));

        rclcpp::QoS qos(100);     
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        robot_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10, 
            std::bind(&MySubscriber::robot_callback, this, std::placeholders::_1));
        cam1_subscriber_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera1/image", qos, std::bind(&MySubscriber::cam1_callback, this, std::placeholders::_1));        
        cam2_subscriber_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera2/image", qos, std::bind(&MySubscriber::cam2_callback, this, std::placeholders::_1));
        cam3_subscriber_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera3/image", qos, std::bind(&MySubscriber::cam3_callback, this, std::placeholders::_1));
        cam4_subscriber_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera4/image", qos, std::bind(&MySubscriber::cam4_callback, this, std::placeholders::_1));
        cam5_subscriber_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera5/image", qos, std::bind(&MySubscriber::cam5_callback, this, std::placeholders::_1));

        // // Load a buffer of transforms
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        // tf_buffer_->setUsingDedicatedThread(true);
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // set the initial pose for navigation
        set_initial_pose();

        std::cout<<"Size of array"<<partposes.size()<<'\n';
    }



private:
    /**
     * @brief stores the waypoints from the parameters 
     */
    std::vector<std::string> wps; 

    /**
     * @brief transformation buffer for transformation wrt other frame
     * 
     */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; 

    /**
     * @brief tfListener_ - Transformation Listener
     * 
     */
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    /**
     * @brief stores the turtlebot in real-time through robot_callback
     * 
     */
    geometry_msgs::msg::Pose robotpose;

    /**
     * @brief subscriber for topic /amcl_pose
     * 
     */
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_subscription_;

    /**
     * @brief subscriber for topic /arucomarkers 
     * 
     */
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscriber_;

    /**
     * @brief subscriber for topic mage/camera1/image 
     * 
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr cam1_subscriber_;

    /**
     * @brief subscriber for topic mage/camera2/image
     * 
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr cam2_subscriber_;

    /**
     * @brief subscriber for topic mage/camera3/image
     * 
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr cam3_subscriber_;

    /**
     * @brief subscriber for topic mage/camera4/image
     * 
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr cam4_subscriber_;

    /**
     * @brief subscriber for topic mage/camera5/image
     * 
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr cam5_subscriber_;

    /**
     * @brief vector containing poses of each part from cam subsribers
     * 
     */
    std::vector<geometry_msgs::msg::Pose> partposes;

    /**
     * @brief sorted vector containing poses in the order from parameters
     * 
     */
    std::vector<geometry_msgs::msg::Pose> waypointposes;

    /**
     * @brief vector containing color and type of all parts
     * 
     */
    std::vector<std::vector<std::string>> parts;

    /**
     * @brief part color as a string
     * 
     */
    std::string p_color;

    /**
     * @brief part type as a string
     * 
     */
    std::string p_type;

    /**
     * @brief initial pose publisher for navigation
     * 
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    /**
     * @brief waypoint publisher to publish waypoints
     * 
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_publisher_;

    /**
     * @brief action client for navigation through poses
     * 
     */
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;

    /**
     * @brief timer_ for publishing waypoints
     * 
     */
    rclcpp::TimerBase::SharedPtr timer_;


    std::string toUpperCase(const std::string& input);
    /**
     * @brief Set the initial pose the turtlebot for navigation
     * 
     */
    void set_initial_pose();

    /**
     * @brief Callback method to subscription topic /amcl_pose
     * 
     * @param msg message is the pose of the turtlebot in real-time while navigation, it is stored in robotpose attribute
     */
    void robot_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief Callback method to subscritption topic mage/camera1/image 
     * 
     * @param msg is the position of the part(battery) wrt to frame camera1_frame
     */
    void cam1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback method to subscritption topic mage/camera2/image 
     * 
     * @param msg is the position of the part(battery) wrt to frame camera2_frame
     */
    void cam2_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback method to subscritption topic mage/camera3/image 
     * 
     * @param msg is the position of the part(battery) wrt to frame camera3_frame
     */
    void cam3_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback method to subscritption topic mage/camera4/image
     * 
     * @param msg is the position of the part(battery) wrt to frame camera4_frame
     */
    void cam4_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback method to subscritption topic mage/camera5/image
     * 
     * @param msg is the position of the part(battery) wrt to frame camera5_frame
     */
    void cam5_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief gets the color and type(string) of the part using ids and assigns to the attributes p_color and p_type 
     * 
     * @param part_color input part_color id
     * @param part_type input part_type id
     */
    void part_data(int part_color, int part_type);

    /**
     * @brief Callback function for the subscription topic /aruco_markers
     * 
     * @param msg is the pose and id of the arucomarker from the RGB camera on the turtlebot
     */
    void subscriber_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * @brief sorts the partposes attribute in the order received from the parameters and adds to a new vector waypointposes
     * 
     */
    void getwaypoints();

    /**
     * @brief calls the getwaypoints() method for sorting and publishes the waypoints from vector waypointposes
     * 
     */
    void publishWaypoints();

    /**
     * @brief callback function for resultfeedback from publishWaypoints() method
     * 
     * @param result input the result_feedback while publishing waypoints
     */
    void resultCallback(const GoalHandleNavigateThroughPoses::WrappedResult &result);



};


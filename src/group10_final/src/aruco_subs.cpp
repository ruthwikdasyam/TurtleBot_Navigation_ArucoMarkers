#include "aruco_subs.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include <functional>




void MySubscriber::subscriber_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{

    
    int markerid;
    RCLCPP_INFO(this->get_logger(), "Marker IDs:");
    for (const auto &marker_id : msg->marker_ids) {
        markerid = marker_id;
        RCLCPP_INFO(this->get_logger(), "  %ld", marker_id);
    }
    
    if(markerid==0){
      RCLCPP_INFO(get_logger(), "Unsubscribing Marker");
      wps.push_back(toUpperCase(this->get_parameter("aruco_0.wp1.color").as_string()));
      wps.push_back(toUpperCase(this->get_parameter("aruco_0.wp2.color").as_string()));
      wps.push_back(toUpperCase(this->get_parameter("aruco_0.wp3.color").as_string()));
      wps.push_back(toUpperCase(this->get_parameter("aruco_0.wp4.color").as_string()));
      wps.push_back(toUpperCase(this->get_parameter("aruco_0.wp5.color").as_string()));
    }
    if(markerid==1){
      wps.push_back(toUpperCase(this->get_parameter("aruco_1.wp1.color").as_string()));
      wps.push_back(toUpperCase(this->get_parameter("aruco_1.wp2.color").as_string()));
      wps.push_back(toUpperCase(this->get_parameter("aruco_1.wp3.color").as_string()));
      wps.push_back(toUpperCase(this->get_parameter("aruco_1.wp4.color").as_string()));
      wps.push_back(toUpperCase(this->get_parameter("aruco_1.wp5.color").as_string()));
    }


    RCLCPP_INFO(get_logger(), "Unsubscribing Marker");

    subscriber_.reset();



    }



void MySubscriber::cam1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    


    geometry_msgs::msg::PoseStamped currpose;
    geometry_msgs::msg::PoseStamped map_pose;


    currpose.header.frame_id = "camera1_frame";   
    currpose.header.stamp = this->get_clock()->now();
    currpose.pose=msg->part_poses[0].pose;

    try {

    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
          "map", currpose.header.frame_id, currpose.header.stamp);

      // Transform the object pose to the world frame
      tf2::doTransform(currpose, map_pose, transformStamped);


    }
    catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    }

    MySubscriber::part_data( msg->part_poses[0].part.color , msg->part_poses[0].part.type);
    RCLCPP_INFO(get_logger(), "%s %s Detected-CAM1", p_color.c_str(), p_type.c_str());

    parts.push_back({p_type, p_color});
    partposes.push_back(map_pose.pose);

    cam1_subscriber_.reset();

    RCLCPP_INFO(get_logger(), "Camera Frame[x, y, z]: [%.3f, %.3f, %.3f]",
                currpose.pose.position.x, currpose.pose.position.y, currpose.pose.position.z);

    RCLCPP_INFO(get_logger(), "Map Frame[x, y, z]: [%.3f, %.3f, %.3f]",
                map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z);

    RCLCPP_INFO(get_logger(), "Unsubscribing Cam1");
    if (partposes.size()==5){
      std::cout<<"started goal"<<'\n';
      publishWaypoints();}
}


void MySubscriber::cam2_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    
    // geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;

    geometry_msgs::msg::PoseStamped currpose;
    geometry_msgs::msg::PoseStamped map_pose;


    currpose.header.frame_id = "camera2_frame";   
    currpose.header.stamp = this->get_clock()->now();
    currpose.pose=msg->part_poses[0].pose;

    try {

    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
          "map", currpose.header.frame_id, currpose.header.stamp);

      // Transform the object pose to the world frame
      tf2::doTransform(currpose, map_pose, transformStamped);


    }
    catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    }

    MySubscriber::part_data( msg->part_poses[0].part.color , msg->part_poses[0].part.type);
    RCLCPP_INFO(get_logger(), "%s %s Detected-CAM2", p_color.c_str(), p_type.c_str());


    parts.push_back({p_type, p_color});
    partposes.push_back(map_pose.pose);

    cam2_subscriber_.reset();

    RCLCPP_INFO(get_logger(), "Camera Frame[x, y, z]: [%.3f, %.3f, %.3f]",
                currpose.pose.position.x, currpose.pose.position.y, currpose.pose.position.z);

    RCLCPP_INFO(get_logger(), "Map Frame[x, y, z]: [%.3f, %.3f, %.3f]",
                map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z);

    RCLCPP_INFO(get_logger(), "Unsubscribing Cam2");
    if (partposes.size()==5){
      std::cout<<"started goal"<<'\n';
      publishWaypoints();}
}


void MySubscriber::cam3_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    
    // geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;

    geometry_msgs::msg::PoseStamped currpose;
    geometry_msgs::msg::PoseStamped map_pose;


    currpose.header.frame_id = "camera3_frame";   
    currpose.header.stamp = this->get_clock()->now();
    currpose.pose=msg->part_poses[0].pose;

    try {

    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
          "map", currpose.header.frame_id, currpose.header.stamp);

      // Transform the object pose to the world frame
      tf2::doTransform(currpose, map_pose, transformStamped);

 
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    }

    MySubscriber::part_data( msg->part_poses[0].part.color , msg->part_poses[0].part.type);
    RCLCPP_INFO(get_logger(), "%s %s Detected-CAM3", p_color.c_str(), p_type.c_str());

    parts.push_back({p_type, p_color});
    partposes.push_back(map_pose.pose);


    cam3_subscriber_.reset();

    RCLCPP_INFO(get_logger(), "Camera Frame[x, y, z]: [%.3f, %.3f, %.3f]",
                currpose.pose.position.x, currpose.pose.position.y, currpose.pose.position.z);

    RCLCPP_INFO(get_logger(), "Map Frame[x, y, z]: [%.3f, %.3f, %.3f]",
                map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z);

    RCLCPP_INFO(get_logger(), "Unsubscribing Cam3");
    if (partposes.size()==5){
      std::cout<<"started goal"<<'\n';
      publishWaypoints();}
}



void MySubscriber::cam4_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    
    // geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;

    geometry_msgs::msg::PoseStamped currpose;
    geometry_msgs::msg::PoseStamped map_pose;


    currpose.header.frame_id = "camera4_frame";   
    currpose.header.stamp = this->get_clock()->now();
    currpose.pose=msg->part_poses[0].pose;

    try {

    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
          "map", currpose.header.frame_id, currpose.header.stamp);

      // Transform the object pose to the world frame
      tf2::doTransform(currpose, map_pose, transformStamped);


    }
    catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    }

    MySubscriber::part_data( msg->part_poses[0].part.color , msg->part_poses[0].part.type);
    RCLCPP_INFO(get_logger(), "%s %s Detected-CAM4", p_color.c_str(), p_type.c_str());

    parts.push_back({p_type, p_color});
    partposes.push_back(map_pose.pose);

    cam4_subscriber_.reset();

      RCLCPP_INFO(get_logger(), "Camera Frame[x, y, z]: [%.3f, %.3f, %.3f]",
                currpose.pose.position.x, currpose.pose.position.y, currpose.pose.position.z);

    RCLCPP_INFO(get_logger(), "Map Frame[x, y, z]: [%.3f, %.3f, %.3f]",
                map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z);

    RCLCPP_INFO(get_logger(), "Unsubscribing Cam4");
    if (partposes.size()==5){
      std::cout<<"started goal"<<'\n';
      publishWaypoints();}

}



void MySubscriber::cam5_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    
    // geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;

    geometry_msgs::msg::PoseStamped currpose;
    geometry_msgs::msg::PoseStamped map_pose;


    currpose.header.frame_id = "camera5_frame";   
    currpose.header.stamp = this->get_clock()->now();
    currpose.pose=msg->part_poses[0].pose;

    try {

    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
          "map", currpose.header.frame_id, currpose.header.stamp);

      // Transform the object pose to the world frame
      tf2::doTransform(currpose, map_pose, transformStamped);


    }
    catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    }

    MySubscriber::part_data( msg->part_poses[0].part.color , msg->part_poses[0].part.type);
    RCLCPP_INFO(get_logger(), "%s %s Detected-CAM5", p_color.c_str(), p_type.c_str());

    parts.push_back({p_type, p_color});
    partposes.push_back(map_pose.pose);

    cam5_subscriber_.reset();

    RCLCPP_INFO(get_logger(), "Camera Frame[x, y, z]: [%.3f, %.3f, %.3f]",
                currpose.pose.position.x, currpose.pose.position.y, currpose.pose.position.z);

    RCLCPP_INFO(get_logger(), "Map Frame[x, y, z]: [%.3f, %.3f, %.3f]",
                map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z);

    RCLCPP_INFO(get_logger(), "Unsubscribing Cam5");
    // std::cout<<"Parts cout: "<<partposes.size()<<'\n';
    if (partposes.size()==5){
      std::cout<<"started goal"<<'\n';
      publishWaypoints();}
}




void MySubscriber::part_data(int part_color, int part_type) {
    // Assign color
    switch (part_color) {
        case mage_msgs::msg::Part::BLUE:
            p_color = "BLUE"; break;
        case mage_msgs::msg::Part::GREEN:   
            p_color = "GREEN"; break;
        case mage_msgs::msg::Part::ORANGE:  
            p_color = "ORANGE"; break;
        case mage_msgs::msg::Part::RED:     
            p_color = "RED"; break;
        case mage_msgs::msg::Part::PURPLE:  
            p_color = "PURPLE"; break;
    }

    // Assign type
    switch (part_type) {
        case mage_msgs::msg::Part::BATTERY:     
            p_type = "BATTERY"; break;
        case mage_msgs::msg::Part::PUMP:        
            p_type = "PUMP"; break;
        case mage_msgs::msg::Part::SENSOR:      
            p_type = "SENSOR"; break;
        case mage_msgs::msg::Part::REGULATOR:   
            p_type = "REGULATOR"; break;
    }

}
   

void MySubscriber::robot_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    robotpose=msg->pose.pose;
    robot_subscription_.reset();
    std::cout<<"Robot Pose noted and Unsubscribed"<<'\n';
}


void MySubscriber::set_initial_pose() {
  auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
  message.header.frame_id = "map";
//   message.header.stamp = this->get_clock()->now();
  message.pose.pose.position.x = 1.0;
  message.pose.pose.position.y = -1.7;
  message.pose.pose.position.z = 0.0;
  message.pose.pose.orientation.x = 0.0;
  message.pose.pose.orientation.y = 0.0;
  message.pose.pose.orientation.z = -1.57;
  message.pose.pose.orientation.w = 1.0;

  initial_pose_pub_->publish(message);
  std::cout<<"initial pose acheived"<<'\n';
}



void MySubscriber::getwaypoints(){
  std::cout<<"Getting Waypoints.."<<'\n';
  std::vector<int> indexes;
  for (auto& i:wps){
    for (size_t j=0; j<parts.size();j++){
      // std::cout<<parts[j][1]<<'\n'; 
      if(i==parts[j][1]){
        indexes.push_back(j);
      }
    }
  }
  std::cout<<"Indexes: "<<indexes.size()<<'\n';
  for(size_t i=0; i<indexes.size(); i++){
    waypointposes.push_back(partposes[indexes[i]]);
    RCLCPP_INFO(get_logger(), "Waypoint %ld: %s", i+1, parts[indexes[i]][1].c_str());
  }

}





 void MySubscriber::publishWaypoints()
  {
    MySubscriber::getwaypoints();
    // Create a goal for the NavigateThroughPoses action
    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses.reserve(partposes.size());
    std::cout<<"Waypoints count: "<<waypointposes.size()<<'\n';
    std::this_thread::sleep_for(std::chrono::seconds(2));
    for (const auto &part : waypointposes)
    {
      auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
      pose_stamped->header.frame_id = "map"; // Set the frame ID, change if needed

      // Set the position
      pose_stamped->pose.position.x = part.position.x;
      pose_stamped->pose.position.y = part.position.y;
      pose_stamped->pose.orientation = part.orientation;

      // Publish the waypoint
      waypoint_publisher_->publish(*pose_stamped);

      // Add the waypoint to the goal
      goal_msg.poses.push_back(*pose_stamped);

      // Wait for a short time before publishing the next waypoint
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
     // Send the goal to the action server
    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&MySubscriber::resultCallback, this, std::placeholders::_1);
    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
    goal_handle_future.wait();
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
    }
    rclcpp::shutdown();
  }


  void MySubscriber::resultCallback(const GoalHandleNavigateThroughPoses::WrappedResult &result)
  {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      rclcpp::shutdown();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  }

std::string MySubscriber::toUpperCase(const std::string& input) {
    std::string result = input;
    for (char& c : result) {
        c = std::toupper(c);
    }
    return result;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MySubscriber>("aruco_subscriber");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

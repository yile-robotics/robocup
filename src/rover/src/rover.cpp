#include <rover/rover.h>

#include <object_server/utilities.h>

namespace rover
{

  Rover::Rover()
  {
  }

  Rover::~Rover()
  {
  }

  bool Rover::initialize(ros::NodeHandle& nh)
  {
    // ros connections
    command_sub_ = nh.subscribe(/*#>>>>TODO: The NAME of the command topic*/, 1, 
                                &Rover::/*#>>>>TODO: The NAME of the callback function*/, 
                                this);

    rover_marker_pub_ = //#>>>>TODO: Create a publisher of type visualization_msgs::Marker and name it "rover"
    
    // init member variables
    pos_b_w_.setZero();
    vel_b_.setZero();
    thrust_b_.setZero();

    // create the marker for visualizing the rover
    rover_marker_ = object_server::create_mesh_marker(
      "package://rover/launch/meshes/preserverance/perseverance.dae",
      1,
      0, 0, 0,
      0.7045553,0, 0, 0.7096491,
      0.3);
    rover_marker_.header.frame_id = "rover";
    rover_marker_.header.stamp = ros::Time::now();

    // get the parameters of this node (see config.yaml)
    std::vector<double> v = {0,0,0};

    if(!ros::param::get(/*#>>>>TODO: PARAMETER NAME*/, v))
      return false;
    M_ = Eigen::Vector3d(v[0], v[1], v[2]).asDiagonal();

    if(!ros::param::get(/*#>>>>TODO: PARAMETER NAME*/, v))
      return false;
    D_ = Eigen::Vector3d(v[0], v[1], v[2]).asDiagonal();

    if(!ros::param::get(/*#>>>>TODO: PARAMETER NAME*/, v))
      return false;
    K_ = Eigen::Vector3d(v[0], v[1], v[2]).asDiagonal();

    return true;
  }

  void Rover::update(ros::Time& time, ros::Duration& duration)
  {
    double dt = duration.toSec();

    // rotation of the rovers body {b} with respect to the world/map {w}
    Eigen::Matrix3d R_b_w = Eigen::Matrix3d::Identity();
    R_b_w.topLeftCorner(2,2) = Eigen::Rotation2Dd(pos_b_w_.z()).matrix();

    // obtain the cooresponding from the given thrust
    Eigen::Vector3d acc_b = dynamic(thrust_b_);

    // integrate velocity into position
    pos_b_w_ += vel_b_*dt + 0.5*acc_b*dt*dt;

    // integrate acceleration into velocity
    vel_b_ += acc_b*dt;

    // update the tf transformation
    tf::StampedTransform T_rover_map;
    T_rover_map.setOrigin(tf::Vector3(pos_b_w_.x(), pos_b_w_.y(), 0.0));

    tf::Quaternion Q_rover_map;
    Q_rover_map.setRPY(0.0, 0.0, pos_b_w_.z());
    T_rover_map.setRotation(Q_rover_map);

    // broadcast the transformation
    T_rover_map.child_frame_id_ = //#>>>>TODO: the child frame name
    T_rover_map.frame_id_ = //#>>>>TODO: the map frame name
    T_rover_map.stamp_ = time;
    broadcaster_.sendTransform(//#>>>>TODO: broadcast to ros

    // publish the visualization
    rover_marker_.header.stamp = time;
    rover_marker_pub_.publish(//#>>>>TODO: publish the rover marker
  }

  Eigen::Vector3d Rover::dynamic(const Eigen::Vector3d& thrust) const
  {
    //#>>>>TODO: Return the rovers acceleration based on the commanded thrust
    //#>>>>TODO: and current velocity vel_b_
    //#>>>>TODO: note Eigen::Matrix3d offers the function inverse() for quadratic matrices
    return //#>>>>TODO: solve for the acceleration
  }

  void Rover::commandCallback(const geometry_msgs::TwistConstPtr& msg)
  {
    //#>>>>TODO: Store the planar part of the msg in the member variable thrust_b_
    //#>>>>TODO: You can check the message definition at http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html 
    //#>>>>TODO: Note that we need the x-y position and the theta angle only (3 dim)

    //#>>>>TODO: You can check the callback with
    //ROS_INFO_STREAM_THROTTLE(0.5, "Rover::commandCallback=" << thrust_b_.transpose());
  }
}
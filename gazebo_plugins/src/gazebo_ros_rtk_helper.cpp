/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Publish GPS and odometry data from Gazebo simulation to ROS.
 */

#include <gazebo_plugins/gazebo_ros_rtk_helper.h>
#include <ignition/math/Rand.hh>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRTKHelper)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosRTKHelper::GazeboRosRTKHelper()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosRTKHelper::~GazeboRosRTKHelper()
{
  this->update_connection_.reset();
  // Finalize the controller
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosRTKHelper::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // save pointers
  this->world_ = _parent->GetWorld();
  this->sdf = _sdf;

  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosRTKHelper::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosRTKHelper::LoadThread()
{
  // load parameters
  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace") + "/";

  if (!this->sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO_NAMED("rtk_helper", "rtk_helper plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0.0;
  }
  else
    this->gaussian_noise_ = this->sdf->Get<double>("gaussianNoise");

  if (!this->sdf->HasElement("bodyFrame"))
  {
    ROS_FATAL_NAMED("rtk_helper", "rtk_helper plugin missing <bodyFrame>, cannot proceed");
    this->link_name_ = "gazebo_link";
  }
  else
    this->link_name_ = this->sdf->Get<std::string>("bodyFrame");

  if (!this->sdf->HasElement("updateRate"))
  {
    ROS_DEBUG_NAMED("rtk_helper", "rtk_helper plugin missing <updateRate>, defaults to 8 Hz");
    this->update_rate_ = 8.0;
  }
  else
    this->update_rate_ = this->sdf->GetElement("updateRate")->Get<double>();

  if (!this->sdf->HasElement("frameId"))
  {
    ROS_INFO_NAMED("rtk_helper", "rtk_helper plugin missing <frameId>, defaults to map");
    this->frame_id_ = "map";
  }
  else
    this->frame_id_ = this->sdf->Get<std::string>("frameId");

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("rtk_helper", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // publish multi queue
  this->pmq.startServiceThread();

  // assert that the body by link_name_ exists
  this->link = boost::dynamic_pointer_cast<physics::Link>(
#if GAZEBO_MAJOR_VERSION >= 8
    this->world_->EntityByName(this->link_name_));
#else
    this->world_->GetEntity(this->link_name_));
#endif
  if (!this->link)
  {
    ROS_FATAL_NAMED("rtk_helper", "gazebo_ros_rtk_helper plugin error: bodyFrame: %s does not exist\n",
      this->link_name_.c_str());
    return;
  }

  this->gps_topic_name_ = "/gazebo/rtk_helper/gps";
  this->odom_topic_name_ = "/gazebo/rtk_helper/odom";

  this->gps_pub_queue = this->pmq.addPub<sensor_msgs::NavSatFix>();
  this->gps_pub_ = this->rosnode_->advertise<sensor_msgs::NavSatFix>(
    this->gps_topic_name_, 1);
  this->odom_pub_queue = this->pmq.addPub<nav_msgs::Odometry>();
  this->gps_pub_ = this->rosnode_->advertise<nav_msgs::Odometry>(
    this->odom_topic_name_, 1);

  // Initialize the controller
#if GAZEBO_MAJOR_VERSION >= 8
  this->last_time_ = this->world_->SimTime();
#else
  this->last_time_ = this->world_->GetSimTime();
#endif

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosRTKHelper::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosRTKHelper::UpdateChild()
{
#if GAZEBO_MAJOR_VERSION >= 8
  common::Time cur_time = this->world_->SimTime();
#else
  common::Time cur_time = this->world_->GetSimTime();
#endif

  // rate control
  if (this->update_rate_ > 0 &&
      (cur_time - this->last_time_).Double() < (1.0 / this->update_rate_))
    return;

  if (this->gps_pub_.getNumSubscribers() > 0 || this->odom_pub_.getNumSubscribers() > 0)
  {
    ignition::math::Pose3d pose;
    ignition::math::Quaterniond rot;
    ignition::math::Vector3d pos;

    // Get Pose/Orientation ///@todo: verify correctness
#if GAZEBO_MAJOR_VERSION >= 8
    pose = this->link->WorldPose();
#else
    pose = this->link->GetWorldPose().Ign();
#endif

    // get Rates
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d lin_vel = this->link->WorldLinearVel();
    ignition::math::Vector3d ang_vel = this->link->WorldAngularVel();
#else
    ignition::math::Vector3d lin_vel = this->link->GetWorldLinearVel().Ign();
    ignition::math::Vector3d ang_vel = this->link->GetWorldAngularVel().Ign();
#endif

    // copy data into odometry message
    this->odom_msg_.header.frame_id = this->frame_id_;
    this->odom_msg_.header.stamp.sec = cur_time.sec;
    this->odom_msg_.header.stamp.nsec = cur_time.nsec;
    this->odom_msg_.child_frame_id = this->link_name_;

    this->odom_msg_.pose.pose.position.x = pose.Pos().X();
    this->odom_msg_.pose.pose.position.y = pose.Pos().Y();
    this->odom_msg_.pose.pose.position.z = pose.Pos().Z();
    this->odom_msg_.pose.pose.orientation.x = pose.Rot().X();
    this->odom_msg_.pose.pose.orientation.y = pose.Rot().Y();
    this->odom_msg_.pose.pose.orientation.z = pose.Rot().Z();
    this->odom_msg_.pose.pose.orientation.w = pose.Rot().W();

    this->odom_msg_.twist.twist.linear.x = lin_vel.X();
    this->odom_msg_.twist.twist.linear.y = lin_vel.Y();
    this->odom_msg_.twist.twist.linear.z = lin_vel.Z();
    this->odom_msg_.twist.twist.angular.x = ang_vel.X();
    this->odom_msg_.twist.twist.angular.y = ang_vel.Y();
    this->odom_msg_.twist.twist.angular.z = ang_vel.Z();

    {
      boost::mutex::scoped_lock lock(this->lock_);
      // publish to ros
      if (this->gps_pub_.getNumSubscribers() > 0)
          this->gps_pub_queue->push(this->gps_msg_, this->gps_pub_);
      if (this->odom_pub_.getNumSubscribers() > 0)
          this->odom_pub_queue->push(this->odom_msg_, this->odom_pub_);
    }

    // save last time stamp
    this->last_time_ = cur_time;
  }
}


//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosRTKHelper::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = ignition::math::Rand::DblUniform();

  // normalized uniform random variable
  double V = ignition::math::Rand::DblUniform();

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}
}

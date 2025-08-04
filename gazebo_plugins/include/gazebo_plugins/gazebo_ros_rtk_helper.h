/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_ROS_RTK_HELPER_HH
#define GAZEBO_ROS_RTK_HELPER_HH

#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class GazeboRosRTKHelper : public ModelPlugin
  {
    /// \brief Constructor
    public: GazeboRosRTKHelper();

    /// \brief Destructor
    public: virtual ~GazeboRosRTKHelper();

    /// \brief Load the controller
    /// \param node XML config node
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void UpdateChild();

    /// \brief The parent World
    private: physics::WorldPtr world_;

    /// \brief The link referred to by this plugin
    private: physics::LinkPtr link;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher gps_pub_;
    private: ros::Publisher odom_pub_;
    private: PubQueue<sensor_msgs::NavSatFix>::Ptr gps_pub_queue;
    private: PubQueue<nav_msgs::Odometry>::Ptr odom_pub_queue;

    /// \brief ros messages
    private: sensor_msgs::NavSatFix gps_msg_;
    private: nav_msgs::Odometry odom_msg_;

    /// \brief store link name
    private: std::string link_name_;

    /// \brief store frame id
    private: std::string frame_id_;

    /// \brief topic names
    private: std::string odom_topic_name_;
    private: std::string gps_topic_name_;

    /// \brief A mutex to lock access to fields
    /// that are used in message callbacks
    private: boost::mutex lock_;

    /// \brief save last_time
    private: common::Time last_time_;

    // rate control
    private: double update_rate_;

    /// \brief: keep initial pose to offset orientation in imu message
    private: ignition::math::Pose3d initial_pose_;

    /// \brief GPS noise
    private: bool gps_noise_;

    // GPS noise model variables
    private: ignition::math::Vector3d gps_bias_ = ignition::math::Vector3d(0,0,0);
    private: ignition::math::Vector3d noise_gps_pos_ = ignition::math::Vector3d(0,0,0);
    private: ignition::math::Vector3d random_walk_gps_ = ignition::math::Vector3d(0,0,0);
    private: double gps_corellation_time_ = 60.0;
    private: double gps_xy_random_walk_ = 2.0;
    private: double gps_z_random_walk_ = 4.0;
    private: double gps_xy_noise_density_ = 2e-4;
    private: double gps_z_noise_density_ = 4e-4;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;
  };
}
#endif

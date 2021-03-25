//=================================================================================================
// Copyright (c) 2018, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>



ros::Publisher g_joint_state_pub;

void fillJointStateMsg(double value, sensor_msgs::JointState& msg)
{
  msg.name.push_back("rh_p12_rn_a");
  msg.name.push_back("rh_r2");
  msg.name.push_back("rh_l1");
  msg.name.push_back("rh_l2");
  msg.name.push_back("gripper");

  msg.position.push_back(value * (1.0/1.06));
  msg.position.push_back(value * (1.0/1.1));
  msg.position.push_back(value * (1.0/1.06));
  msg.position.push_back(value * (1.0/1.1));
  msg.position.push_back(value);
}

void rhJointStateCb(const sensor_msgs::JointState::ConstPtr& msg)
{
  sensor_msgs::JointState rh_joint_state;

  for (size_t i = 0; i < msg->name.size(); i++)
  {
    if (msg->name[i] == "gripper")
      fillJointStateMsg(msg->position[i], rh_joint_state);
  }

  if (!rh_joint_state.name.empty())
  {
    rh_joint_state.header = msg->header;
    g_joint_state_pub.publish(rh_joint_state);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_gripper_publisher");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Subscriber joint_state_sub = nh.subscribe("/robotis/present_joint_states", 5, rhJointStateCb);
  g_joint_state_pub = nh.advertise<sensor_msgs::JointState>(pnh.param("target", std::string("joint_states")), 0);

  ros::spin();

  return 0;
}
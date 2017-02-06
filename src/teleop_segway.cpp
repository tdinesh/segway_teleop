/*
 * teleop_segway
 * Copyright (c) 2016, University of Pennsylvania
 *
 * teleop_pr2
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

///\author Kevin Watts
///\brief Converts joystick commands on /joy to commands to PR2 base, spine, head

///\author Dinesh Thakur
///\brief Converts joystick commands on /joy to commands to Segway base with limits

#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <topic_tools/MuxSelect.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#define sign(X) (X > 0) ? 1 : ((X < 0) ? -1 : 0)

using namespace std;

class TeleopSegway
{
   public:
  geometry_msgs::Twist cmd;

  double req_vx, req_vy, req_vw;
  double max_vx, max_vy, max_vw, max_vx_run, max_vy_run, max_vw_run;
  int axis_vx, axis_vy, axis_vw;
  int deadman_button, run_button, head_button;
  bool deadman_no_publish_, head_publish;

  double acc_step_v_, acc_step_w_; //step sizes for v and w
  float old_command_x_, old_command_z_;

  bool deadman_, cmd_head;
  bool use_mux_, last_deadman_;
  std::string last_selected_topic_;

  ros::Time last_recieved_joy_message_time_;
  ros::Duration joy_msg_timeout_;

  ros::NodeHandle n_, n_private_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::ServiceClient mux_client_;

  TeleopSegway(bool deadman_no_publish = false) :
    max_vx(0.6), max_vy(0.6), max_vw(0.8),
    max_vx_run(0.6), max_vy_run(0.6), max_vw_run(0.8),
    deadman_no_publish_(deadman_no_publish), deadman_(false), cmd_head(false),
    use_mux_(false), last_deadman_(false),
    n_private_("~")
  { }

  void init()
  {
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;


    //parameters for interaction with a mux on cmd_vel topics
    n_private_.param("use_mux", use_mux_, false);

    n_private_.param("max_vx", max_vx, max_vx);
    n_private_.param("max_vy", max_vy, max_vy);
    n_private_.param("max_vw", max_vw, max_vw);

    // Set max speed while running
    n_private_.param("max_vx_run", max_vx_run, max_vx_run);
    n_private_.param("max_vy_run", max_vy_run, max_vy_run);
    n_private_.param("max_vw_run", max_vw_run, max_vw_run);

    n_private_.param("axis_vx", axis_vx, 3);
    n_private_.param("axis_vw", axis_vw, 0);
    n_private_.param("axis_vy", axis_vy, 2);

    n_private_.param("deadman_button", deadman_button, 0);
    n_private_.param("run_button", run_button, 0);
    n_private_.param("head_button", head_button, 0);

  	n_private_.param("acc_step_v", acc_step_v_, 0.1);
  	n_private_.param("acc_step_w", acc_step_w_, 0.1);

	  old_command_x_ = 0.0; old_command_z_ = 0.0;


    double joy_msg_timeout;
      n_private_.param("joy_msg_timeout", joy_msg_timeout, 0.5); //default to 0.5 seconds timeout
    if (joy_msg_timeout <= 0)
	  {
	    joy_msg_timeout_ = ros::Duration().fromSec(9999999);//DURATION_MAX;
	    ROS_DEBUG("joy_msg_timeout <= 0 -> no timeout");
	  }
    else
	  {
	    joy_msg_timeout_.fromSec(joy_msg_timeout);
	    ROS_DEBUG("joy_msg_timeout: %.3f", joy_msg_timeout_.toSec());
	  }

    ROS_DEBUG("max_vx: %.3f m/s\n", max_vx);
    ROS_DEBUG("max_vy: %.3f m/s\n", max_vy);
    ROS_DEBUG("max_vw: %.3f deg/s\n", max_vw*180.0/M_PI);

    ROS_DEBUG("max_vx_run: %.3f m/s\n", max_vx_run);
    ROS_DEBUG("max_vy_run: %.3f m/s\n", max_vy_run);
    ROS_DEBUG("max_vw_run: %.3f deg/s\n", max_vw_run*180.0/M_PI);

    ROS_DEBUG("axis_vx: %d\n", axis_vx);
    ROS_DEBUG("axis_vy: %d\n", axis_vy);
    ROS_DEBUG("axis_vw: %d\n", axis_vw);

    ROS_DEBUG("deadman_button: %d\n", deadman_button);
    ROS_DEBUG("run_button: %d\n", run_button);
    ROS_DEBUG("head_button: %d\n", head_button);
    ROS_DEBUG("joy_msg_timeout: %f\n", joy_msg_timeout);

    vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub_ = n_.subscribe("joy", 10, &TeleopSegway::joy_cb, this);

    //if we're going to use the mux, then we'll subscribe to state changes on the mux
    if(use_mux_){
      ros::NodeHandle mux_nh("mux");
      mux_client_ = mux_nh.serviceClient<topic_tools::MuxSelect>("select");
    }
  }

  ~TeleopSegway() { }

  /** Callback for joy topic **/
  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    //Record this message reciept
    last_recieved_joy_message_time_ = ros::Time::now();

    deadman_ = (((unsigned int)deadman_button < joy_msg->buttons.size()) && joy_msg->buttons[deadman_button]);
    if (!deadman_)
      return;

    cmd_head = (((unsigned int)head_button < joy_msg->buttons.size()) && joy_msg->buttons[head_button]);

    // Base
    bool running = (((unsigned int)run_button < joy_msg->buttons.size()) && joy_msg->buttons[run_button]);
    double vx = running ? max_vx_run : max_vx;
    double vy = running ? max_vy_run : max_vy;
    double vw = running ? max_vw_run : max_vw;

    if((axis_vx >= 0) && (((unsigned int)axis_vx) < joy_msg->axes.size()) && !cmd_head)
      req_vx = joy_msg->axes[axis_vx] * vx;
    else
      req_vx = 0.0;
    if((axis_vy >= 0) && (((unsigned int)axis_vy) < joy_msg->axes.size()) && !cmd_head)
      req_vy = joy_msg->axes[axis_vy] * vy;
    else
      req_vy = 0.0;
    if((axis_vw >= 0) && (((unsigned int)axis_vw) < joy_msg->axes.size()) && !cmd_head)
      req_vw = joy_msg->axes[axis_vw] * vw;
    else
      req_vw = 0.0;

    // Enforce max/mins for velocity
    // Joystick should be [-1, 1], but it might not be
    req_vx = max(min(req_vx, vx), -vx);
    req_vy = max(min(req_vy, vy), -vy);
    req_vw = max(min(req_vw, vw), -vw);

  }

  void send_cmd_vel()
  {
    if(!deadman_ && ( fabs(old_command_x_) > 0 || fabs(old_command_z_) > 0))
    {
      ROS_INFO("stopping segway");

      //stop with acceleration limits
      cmd.linear.x = old_command_x_;
      cmd.angular.z = old_command_z_;

    	double diff_x = -old_command_x_;
    	double diff_z = -old_command_z_;

      //ROS_INFO("old_x: %f, old_z:%f, step:%f, b1: %d, b2: %d", old_command_x_, old_command_z_, acc_step_v_,  fabs(old_command_x_) > 0 , fabs(old_command_z_) > 0);
	    if( fabs(diff_x) > acc_step_v_)
		    cmd.linear.x = old_command_x_ + float(sign(diff_x))*acc_step_v_;
		  else
		    cmd.linear.x = 0.0;

	    if( fabs(diff_z) > acc_step_w_)
		    cmd.angular.z = old_command_z_ + float(sign(diff_z))*acc_step_w_;
 		  else
		    cmd.angular.z = 0.0;

	    //ROS_INFO("comm_x: %f, %f, %f, sum:%f, te %f", cmd.linear.x, old_command_x_, float(sign(diff_x))*acc_step_v_, (old_command_x_ ) + float(sign(diff_x))*acc_step_v_ );
  		//ROS_INFO("joy_acc_step: new_x: %f, old_x: %f, comm_x %f\n", cmd.linear.x, old_command_x_, cmd.linear.x);

	    old_command_x_ = cmd.linear.x;
	    old_command_z_ = cmd.angular.z;

      vel_pub_.publish(cmd);

    }

    if(deadman_  &&
       last_recieved_joy_message_time_ + joy_msg_timeout_ > ros::Time::now())
    {
      //check if we need to switch the mux to our topic for teleop
      if(use_mux_ && !last_deadman_){
        topic_tools::MuxSelect select_srv;
        select_srv.request.topic = vel_pub_.getTopic();
        if(mux_client_.call(select_srv)){
          last_selected_topic_ = select_srv.response.prev_topic;
          ROS_DEBUG("Setting mux to %s for teleop", select_srv.request.topic.c_str());
        }
        else{
          ROS_ERROR("Failed to call select service %s on mux. Are you sure that it is up and connected correctly to the teleop node?", mux_client_.getService().c_str());
        }
      }

      // Base

      //Enfore step limits
      cmd.linear.x = req_vx;
      cmd.linear.y = req_vy;
      cmd.angular.z = req_vw;

    	double diff_x = cmd.linear.x - old_command_x_;
    	double diff_z = cmd.angular.z - old_command_z_;

      //ROS_INFO("new_x: %f, old_x: %f, diff: %f sign: %d, step:%f", req_vx, old_command_x_, diff_x, sign(diff_x), acc_step_v_);
	    if( fabs(diff_x) > acc_step_v_)
		    cmd.linear.x = old_command_x_ + float(sign(diff_x))*acc_step_v_;

	    if( fabs(diff_z) > acc_step_w_)
		    cmd.angular.z = old_command_z_ + float(sign(diff_z))*acc_step_w_;

	    //ROS_INFO("comm_x: %f, %f, %f, sum:%f, te %f", cmd.linear.x, old_command_x_, float(sign(diff_x))*acc_step_v_, (old_command_x_ ) + float(sign(diff_x))*acc_step_v_ );
  		//ROS_INFO("joy_acc_step: new_x: %f, old_x: %f, comm_x %f\n", cmd.linear.x, old_command_x_, cmd.linear.x);

      vel_pub_.publish(cmd);

	    old_command_x_ = cmd.linear.x;
	    old_command_z_ = cmd.angular.z;

    }
    else
    {
      //make sure to set the mux back to whatever topic it was on when we grabbed it if the deadman has just toggled
      if(use_mux_ && last_deadman_){
        topic_tools::MuxSelect select_srv;
        select_srv.request.topic = last_selected_topic_;
        if(mux_client_.call(select_srv)){
          ROS_DEBUG("Setting mux back to %s", last_selected_topic_.c_str());
        }
        else{
          ROS_ERROR("Failed to call select service %s on mux. Are you sure that it is up and connected correctly to the teleop node?", mux_client_.getService().c_str());
        }
      }

      // Publish zero commands iff deadman_no_publish is false
      cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
      if (!deadman_no_publish_)
      {
        // Base
        vel_pub_.publish(cmd);
      }

    }

    //make sure we store the state of our last deadman
    last_deadman_ = deadman_;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_segway");
  const char* opt_no_publish    = "--deadman_no_publish";

  bool no_publish = false;
  for(int i=1;i<argc;i++)
  {
    if(!strncmp(argv[i], opt_no_publish, strlen(opt_no_publish)))
      no_publish = true;
  }

  TeleopSegway teleop_segway(no_publish);
  teleop_segway.init();

  ros::Rate pub_rate(20);

  while (teleop_segway.n_.ok())
  {
    ros::spinOnce();
    teleop_segway.send_cmd_vel();
    pub_rate.sleep();
  }

  exit(0);
  return 0;
}


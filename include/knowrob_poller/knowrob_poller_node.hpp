/* Copyright (c) 2016, Georg Bartels <georg.bartels@cs.uni-bremen.de>
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Universität Bremen nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.

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

#ifndef KNOWROB_POLLER_KNOWROB_POLLER_NODE_HPP
#define KNOWROB_POLLER_KNOWROB_POLLER_NODE_HPP

#include <exception>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <knowrob_poller/ros_utils.hpp>
#include <json_prolog/prolog.h>
#include <iai_pepper_demo_msgs/PepperComm.h>

namespace knowrob_poller
{
  class KnowrobPoller
  {
    public:
      KnowrobPoller(const ros::NodeHandle& nh) : nh_(nh) {}
      ~KnowrobPoller() {}

      void start()
      {
        double period = knowrob_poller::readParam<double>(nh_, "periodicity");
        dummy_mode_ = knowrob_poller::readParam<bool>(nh_, "dummy_mode");
        string_pub_ = nh_.advertise<std_msgs::String>("message", 1);
        point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("point", 1);
        timer_ = nh_.createTimer(ros::Duration(period), &KnowrobPoller::callback, this);
        pepper_client_ = nh_.serviceClient<iai_pepper_demo_msgs::PepperComm>("/pepper/comm");
        if (!pepper_client_.waitForExistence(ros::Duration(5)))
          throw std::runtime_error("Could not reach pepper server.");
      }

    private:
      ros::NodeHandle nh_;
      ros::Publisher string_pub_, point_pub_;
      ros::ServiceClient pepper_client_;
      ros::Timer timer_;
      json_prolog::Prolog prolog_;
      bool dummy_mode_;

      void send_speech(const std::string& speech)
      {
        std_msgs::String msg;
        msg.data = speech;
        string_pub_.publish(msg);

        iai_pepper_demo_msgs::PepperComm r;
        r.request.speak = speech;
        pepper_client_.call(r);
      }

      void send_point(double x, double y, double z)
      {
        geometry_msgs::PointStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.point.x = x;
        msg.point.y = y;
        msg.point.z = z;
        point_pub_.publish(msg);
      }

      void callback(const ros::TimerEvent& e)
      {
        std::string query_string;

        if (dummy_mode_)
          query_string = "rdf_instance_from_class(knowrob:'GraspingSomething', _A), create_timepoint(0.0, _I), rdf_assert(_A, knowrob:'startTime', _I), comment(What, [X, Y, Z]).";
        else
          query_string = "comment(What, _).";

        json_prolog::PrologQueryProxy bdgs = prolog_.query(query_string);

        if (bdgs.begin() == bdgs.end())
          send_speech("Prolog query returned no bindings.");
        else
        {
          json_prolog::PrologBindings bdg = *(bdgs.begin());
          send_speech(bdg["What"]);
//          send_point(std::stod(bdg["X"].toString()), 
//              std::stod(bdg["Y"].toString()), std::stod(bdg["Z"].toString()));
        }
      }
  };
}

#endif // KNOWROB_POLLER_KNOWROB_POLLER_NODE_HPP

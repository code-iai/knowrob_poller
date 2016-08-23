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

#include <ros/ros.h>
#include <knowrob_poller/ros_utils.hpp>

namespace knowrob_poller
{
  class KnowrobPoller
  {
    public:
      KnowrobPoller(const ros::NodeHandle& nh) : nh_(nh) {}
      ~KnowrobPoller() {}

      void start()
      {
        ros::Duration period = ros::Duration(knowrob_poller::readParam<double>(nh_, "periodicity"));
        ROS_INFO_STREAM("period: " << period);
        timer_ = nh_.createTimer(period, &KnowrobPoller::callback, this);
        // TODO: complete me
      }

    private:
      ros::NodeHandle nh_;
      ros::Timer timer_;

      void callback(const ros::TimerEvent& e)
      {
        // TODO: complete me
        ROS_INFO("Called.");

      }
  };
}

#endif // KNOWROB_POLLER_KNOWROB_POLLER_NODE_HPP

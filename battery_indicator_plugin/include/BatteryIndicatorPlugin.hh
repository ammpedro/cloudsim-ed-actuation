/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _GAZEBO_BATTERY_INDICATOR_PLUGIN_HH_
#define _GAZEBO_BATTERY_INDICATOR_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/common.hh>

#include <battery_plugin/Battery.h>

namespace gazebo
{
class BatteryIndicatorPlugin : public VisualPlugin
{   
    public: BatteryIndicatorPlugin();
    public: virtual  ~BatteryIndicatorPlugin();

    public: void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/);

    private: void DeferredLoad();

    public: void chargeCallback(const battery_plugin::Battery::ConstPtr& cmd_msg); 
    
    public: void OnUpdate();

    private: common::Color color_indicator;

    private: rendering::VisualPtr modelviz;
    
    private: event::ConnectionPtr updateConnection;
 
    private: ros::NodeHandle* rosNode;
    private: ros::Subscriber batt_subscriber;

    private: boost::thread deferredLoadThread;
};
}

#endif 

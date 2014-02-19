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


#include "BatteryIndicatorPlugin.hh"
#include "battery_plugin/BatteryModelPlugin.hh"

#include <common/common.hh>
#include <ros/ros.h>


using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(BatteryIndicatorPlugin)

BatteryIndicatorPlugin::BatteryIndicatorPlugin()
{
    this->rosNode = NULL;
}

/////////////////////////////////////////////////
BatteryIndicatorPlugin::~BatteryIndicatorPlugin()
{
    delete this->rosNode;
}

/////////////////////////////////////////////////
void BatteryIndicatorPlugin::Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    this->modelviz = _parent;

  std::map<std::string, std::string> m;
  ros::init(m ,"batt_indicator" );

    this->color_indicator.Set(0.0, 0.0, 0.0);

  this->deferredLoadThread =
    boost::thread(boost::bind(&BatteryIndicatorPlugin::DeferredLoad, this));

    this->updateConnection = event::Events::ConnectPreRender(boost::bind(&BatteryIndicatorPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void BatteryIndicatorPlugin::DeferredLoad()
{

  gzwarn << "Battery Indicator Loaded..." << std::endl;

  if (!ros::isInitialized())
  {
    sleep(50);
    gzerr << "Not loading Battery Indicator plugin because ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:" << std::endl
          << "  gazebo -s libgazebo_ros_api_plugin.so" << std::endl;
    return;
  }
  // subscribe
  this->rosNode = new ros::NodeHandle("~");    
  this->batt_subscriber = this->rosNode->subscribe<battery_plugin::Battery>("/battery_msg", 1, boost::bind(&BatteryIndicatorPlugin::chargeCallback, this, _1));
  ros::spin();
}

/////////////////////////////////////////////////
void BatteryIndicatorPlugin::chargeCallback(
      const battery_plugin::Battery::ConstPtr& _msg) 
{
    //gzwarn << "charge call back" << std::endl;

    battery_plugin::Battery batt_msg = *_msg;
    double current_charge = batt_msg.current_batt_charge;

    if (current_charge >= 80.0)
        this->color_indicator.Set(0.0, 1.0, 0.0);
    else if (current_charge < 80.0 && current_charge > 60.0)
        this->color_indicator.Set(0.5, 0.9, 0.35);
    else if (current_charge <= 60.0 && current_charge > 40.0)
        this->color_indicator.Set(1.0, 0.5, 0.0);
    else if (current_charge <= 40.0 && current_charge > 20.0)
        this->color_indicator.Set(1.0, 0.25, 0.0);
    else
        this->color_indicator.Set(1.0, 0.0, 0.0);
}

/////////////////////////////////////////////////
void BatteryIndicatorPlugin::OnUpdate()
{
    static float r=0.0, increment=0.01;
    if (r > 0.5) {
        increment = -0.01;
        r = 0.5;
    }
    if (r < 0.0) {
        r = 0.0;
        increment = 0.01;
    }

    //
    common::Color c(r, r, r);
    this->modelviz->SetAmbient(this->color_indicator);
    this->modelviz->SetDiffuse(this->color_indicator);
    this->modelviz->SetEmissive(c);
    //
    r += increment;
}

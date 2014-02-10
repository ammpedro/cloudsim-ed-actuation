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

#include <string>
#include <vector>
#include <stdlib.h>
#include <time.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "BatteryModelPlugin.hh"

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/util/util.hh>
#include <gazebo/physics/physics.hh>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryModelPlugin)

/////////////////////////////////////////////////
BatteryModelPlugin::BatteryModelPlugin()
{
        gzwarn << "Created BatteryModelPlugin\n";
}

/////////////////////////////////////////////////
void BatteryModelPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  gzwarn << "In Load()\n";

  this->model = _model;
  this->world = this->model->GetWorld();
  this->prevBatteryTime = common::Time(0,0);
  this->elapsedTimeSim = common::Time(0,0);
  this->prevUpdateTime = common::Time(0,0);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  // check for left and right wheel
  if (!_sdf->HasElement("left_wheel"))
    gzerr << "BatteryModel plugin missing <left_wheel> element in SDF file\n";

  if (!_sdf->HasElement("right_wheel"))
    gzerr << "BatteryModel plugin missing <right_wheel> element in SDF file\n";

  // get left and right wheels
  this->leftWheel = _model->GetJoint(
      _sdf->GetElement("left_wheel")->Get<std::string>());
  this->rightWheel = _model->GetJoint(
      _sdf->GetElement("right_wheel")->Get<std::string>());

  // set torque value
  if (_sdf->HasElement("torque"))
    this->torque = _sdf->GetElement("torque")->Get<double>();

  else
  {
    gzwarn << "No torque value set. (default) Torque = 5.0.\n";
    this->torque = 5.0;
  }

  // check and set battery parameters
  if (!_sdf->HasElement("initial_charge"))
    gzerr << "BatteryModel plugin missing <initial_charge> element in SDF file\n";
  else
  {  
    this->initCharge = _sdf->GetElement("initial_charge")->Get<double>();
    this->battCharge = this->initCharge;
  }

  if (!_sdf->HasElement("rated_capacity"))
    gzerr << "BatteryModel plugin missing <rated_capacity> element in SDF file\n";
  else
  {  
    this->ratedCapacity = _sdf->GetElement("rated_capacity")->Get<double>();
    this->battCapacity = this->ratedCapacity;
}

  if (!_sdf->HasElement("nominal_voltage"))
    gzerr << "BatteryModel plugin missing <nominal_voltage> element in SDF file\n";
  else
    this->nominalVoltage = _sdf->GetElement("nominal_voltage")->Get<double>();


  // check wheels
  if (!this->leftWheel)
    gzerr << "Unable to find left wheel["
          << _sdf->GetElement("left_wheel")->Get<std::string>() << "]\n";
  if (!this->rightWheel)
    gzerr << "Unable to find right wheel["
          << _sdf->GetElement("right_wheel")->Get<std::string>() << "]\n";



  // battery charge messages
  if (_sdf->HasElement("battery_file"))
    this->batteryFilePath =
      boost::filesystem::path(_sdf->Get<std::string>("battery_file"));
  else
  {
    // Get the user's home directory
    // \todo getenv is not portable, and there is no generic cross-platform
    // method. Must check OS and choose a method
    char *homePath = getenv("HOME");

    if (!homePath)
      this->batteryFilePath = boost::filesystem::path("/tmp/gazebo");
    else
      this->batteryFilePath = boost::filesystem::path(homePath);

    this->batteryFilePath /= ".gazebo";
    this->batteryFilePath /= "battery";
    this->batteryFilePath /= this->world->GetName() + ".battery";
  }

  // Create the battery directory if needed
  if (!boost::filesystem::exists(this->batteryFilePath.parent_path()))
    boost::filesystem::create_directories(this->batteryFilePath.parent_path());
  // Open the battery file for writing
  this->batteryFileStream.open(this->batteryFilePath.string().c_str(),
                             std::fstream::out);
  if (!this->batteryFileStream.is_open())
  {
    gzerr << "Failed to open battery file :" << this->batteryFilePath <<
      std::endl;
    return;
  }
  gzlog << "Writing battery data to " << this->batteryFilePath << std::endl;

  this->batteryFileStream << "# Battery data for robot model " <<
    this->world->GetName() << std::endl;
  this->runStartTimeWall = common::Time::GetWallTime();
  const time_t timeSec = this->runStartTimeWall.sec;
  this->batteryFileStream << "# Started at: " <<
    std::fixed << std::setprecision(3) <<
    this->runStartTimeWall.Double() << "; " <<
    ctime(&timeSec);
  this->batteryFileStream << "# Format: " << std::endl;
  this->batteryFileStream << "# wallTime(sec),simTime(sec),"
    "wallTimeElapsed(sec),simTimeElapsed(sec)" << std::endl;


  this->trajSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/pose_trajectory",
      &BatteryModelPlugin::OnPoseTrajectoryMsg, this);

  this->deferredLoadThread =
    boost::thread(boost::bind(&BatteryModelPlugin::DeferredLoad, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&BatteryModelPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void BatteryModelPlugin::Init()
{
  gzwarn << "In Init()\n";

  // compute wheelbase from model
  this->wheelSeparation = this->leftWheel->GetAnchor(0).Distance(
      this->rightWheel->GetAnchor(0));

  // compute wheel radius from left wheel
  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->leftWheel->GetChild());
  math::Box bb = parent->GetBoundingBox();
  // This assumes that the largest dimension of the wheel is the diameter
  this->wheelRadius = bb.GetSize().GetMax() * 0.5;

  // compute
  
}

/////////////////////////////////////////////////
void BatteryModelPlugin::DeferredLoad()
{
/*
  std::map<std::string, std::string> m;
  ros::init(m ,"actuation" );
  
  //ros::NodeHandle nh;
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading VRC scoring plugin because ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // publish multi queue
  this->pmq->startServiceThread();

  this->pubBatteryMsgQueue = this->pmq->addPub<battery_plugin::Battery>();
  this->pubBatteryMsg = this->rosNode->advertise<battery_plugin::Battery>(
    "battery_msg", 1, true);
*/
}

/////////////////////////////////////////////////
void BatteryModelPlugin::WriteBatteryState(const common::Time &_simTime,
  const common::Time &_wallTime, const std::string &_msg, bool _force)
{
  // Write at 1Hz
  if (!_force && (_simTime - this->prevBatteryTime).Double() < 1.0)
    return;

  // If we're being forced, that means that something interesting happened.
  // Also force the gazebo state logger to write.
  if (_force)
  {
     gzdbg << "BatteryModelPlugin forcing LogRecord to write" << std::endl;
     util::LogRecord::Instance()->Notify();
  }

  if (!this->batteryFileStream.is_open())
  {
    gzerr << "Battery file stream is no longer open:" << this->batteryFilePath <<
      std::endl;
    return;
  }

    common::Time elapsedTimeWall;
    if (this->stopTimeWall != common::Time::Zero)
      elapsedTimeWall = stopTimeWall - startTimeWall;
    else if (this->startTimeWall != common::Time::Zero)
      elapsedTimeWall = _wallTime - this->startTimeWall;

    common::Time runElapsedTimeWall = _wallTime - this->runStartTimeWall;

  // Output message in json format
  this->batteryFileStream << std::fixed << std::setprecision(3)
    << "{"
    << "\"wall_time\": " << runElapsedTimeWall.Double() << ", "
    << "\"sim_time\": " << _simTime.Double() << ", "
    << "\"wall_time_elapsed\": " << elapsedTimeWall.Double() << ", "
    << "\"sim_time_elapsed\": " << elapsedTimeSim.Double() << ", "
    << "\"message\": \"" << _msg << "\""
    << "}" << std::endl;

  // Also publish via ROS
  battery_plugin::Battery rosBatteryMsg;
  rosBatteryMsg.wall_time = ros::Time(runElapsedTimeWall.Double());
  rosBatteryMsg.sim_time = ros::Time(_simTime.Double());
  rosBatteryMsg.wall_time_elapsed = ros::Time(elapsedTimeWall.Double());
  rosBatteryMsg.sim_time_elapsed = ros::Time(elapsedTimeSim.Double());
  rosBatteryMsg.message = _msg;

  this->pubBatteryMsgQueue->push(rosBatteryMsg, this->pubBatteryMsg);

  this->prevBatteryTime = _simTime;

}

/////////////////////////////////////////////////
void BatteryModelPlugin::OnPoseTrajectoryMsg(
    ConstPoseTrajectoryPtr &/*_msg*/)
{
}

/////////////////////////////////////////////////
void BatteryModelPlugin::OnUpdate()
{
  common::Time simTime =  this->model->GetWorld()->GetSimTime();
  common::Time wallTime = common::Time::GetWallTime();

  std::string BatteryMsg = "test";
  bool forceLogBattery = false;  

  this->leftWheel->SetForce(0, 1);
  this->rightWheel->SetForce(0, 1);
  /* forward until battery depleted
  if (this->battCharge > 0.0)
  {
    double leftVelDesired = 0.5;
    double rightVelDesired = 0.5;

    this->leftWheel->SetForce(0, leftVelDesired);
    this->rightWheel->SetForce(0, rightVelDesired);
  
  }
*/  


  // temporary, assumed 100% charge; 0.005% per update
  this->battCharge = this->battCharge - 0.005;

  // write battery status messages. null pointer issues 
  if (!fmod(simTime.Double(), 1000))
  {
    gzwarn << "Battery: " << this->battCharge  << std::endl;    
    //this->WriteBatteryState(simTime, wallTime, BatteryMsg, forceLogBattery);
  }


}

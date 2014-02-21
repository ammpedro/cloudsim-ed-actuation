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

#include <string>
#include <vector>
#include <stdlib.h>
#include <time.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/util/util.hh>
#include <gazebo/physics/physics.hh>

#include "GateScoringPlugin.hh"

using namespace gazebo;

/////////////////////////////////////////////////
GateScoringPlugin::GateScoringPlugin()
 : postCompletionQuietTime(5.0)
{
  this->pmq = new PubMultiQueue();
  this->rosNode = NULL;
}

/////////////////////////////////////////////////
GateScoringPlugin::~GateScoringPlugin()
{
  delete this->pmq;
  delete this->rosNode;

  // Be sure to write the final score data before quitting
  if (this->scoreFileStream.is_open())
  {
    this->WriteScore(this->world->GetSimTime(), 
                     common::Time::GetWallTime(),
                     "Shutting down", true);
  }
  // Also force the Gazebo state logger to write
  util::LogRecord::Instance()->Notify();
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->deferredLoadThread.join();
}

/////////////////////////////////////////////////
void GateScoringPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  // Which type of world are we scoring?
  this->world = _world;

  if (!this->FindGates())
    return;

  this->prevScoreTime = common::Time(0,0);

  this->runCompleted = false;
  this->completionScore = 0;

  if (_sdf->HasElement("score_file"))
    this->scoreFilePath =
      boost::filesystem::path(_sdf->Get<std::string>("score_file"));
  else
  {
    // Get the user's home directory
    // \todo getenv is not portable, and there is no generic cross-platform
    // method. Must check OS and choose a method
    char *homePath = getenv("HOME");

    if (!homePath)
      this->scoreFilePath = boost::filesystem::path("/tmp/gazebo");
    else
      this->scoreFilePath = boost::filesystem::path(homePath);

    this->scoreFilePath /= ".gazebo";
    this->scoreFilePath /= "scores";
    this->scoreFilePath /= this->world->GetName() + ".score";
  }

  // Create the score directory if needed
  if (!boost::filesystem::exists(this->scoreFilePath.parent_path()))
    boost::filesystem::create_directories(this->scoreFilePath.parent_path());
  // Open the score file for writing
  this->scoreFileStream.open(this->scoreFilePath.string().c_str(),
                             std::fstream::out);
  if (!this->scoreFileStream.is_open())
  {
    gzerr << "Failed to open score file :" << this->scoreFilePath <<
      std::endl;
    return;
  }
  gzlog << "Writing score data to " << this->scoreFilePath << std::endl;

  this->scoreFileStream << "# Score data for world " <<
    this->world->GetName() << std::endl;
  this->runStartTimeWall = common::Time::GetWallTime();
  const time_t timeSec = this->runStartTimeWall.sec;
  this->scoreFileStream << "# Started at: " <<
    std::fixed << std::setprecision(3) <<
    this->runStartTimeWall.Double() << "; " <<
    ctime(&timeSec);
  this->scoreFileStream << "# Format: " << std::endl;
  this->scoreFileStream << "# wallTime(sec),simTime(sec),"
    "wallTimeElapsed(sec),simTimeElapsed(sec),completionScore(count),"
    "falls(count)" << std::endl;

  std::string leaderboardFile = "leaderboard.log";  
  this->leaderboardFilePath = boost::filesystem::path(this->scoreFilePath);
  this->leaderboardFilePath.remove_leaf() /= leaderboardFile;
  // Open the score file for writing
  this->leaderboardFileStream.open(this->leaderboardFilePath.string().c_str(),
                             std::fstream::out | std::fstream::app);
  if (!this->leaderboardFileStream.is_open())
  {
    gzerr << "Failed to open leaderboard file :" << this->leaderboardFilePath <<
      std::endl;
    return;
  }
  this->username = "guest";

  this->deferredLoadThread =
    boost::thread(boost::bind(&GateScoringPlugin::DeferredLoad, this));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GateScoringPlugin::OnUpdate, this, _1));
}

////////////////////////////////////////////////////////////////////////////////
void GateScoringPlugin::DeferredLoad()
{ 

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

  this->pubScoreQueue = this->pmq->addPub<scoring_plugins::Score>();
  this->pubScore = this->rosNode->advertise<scoring_plugins::Score>(
    "vrc_score", 1, true);

// Everybody needs TurtleBot.
  this->turtleBot.reset();
  while (!this->turtleBot)
  {
    this->turtleBot = this->world->GetModel("basic_robot");
    gzwarn << "Waiting for turtlebot" << std::endl;
    sleep(1.0);
    //gzerr << "Failed to find turtlebot" << std::endl;
    
  }
  this->initialPose = this->turtleBot->GetWorldPose();
  gzwarn << "Found turtlebot" << std::endl;
}

/////////////////////////////////////////////////
void GateScoringPlugin::Reset()
{
  // Be sure to write the final score data before resetting
  if (this->scoreFileStream.is_open())
  {
    this->WriteScore(this->currentSimTime,  
                     common::Time::GetWallTime(),
                     "Reset", true);
    this->scoreFileStream.close();
  }
  // Also force the Gazebo state logger to write
  util::LogRecord::Instance()->Notify();

  if (!this->runCompleted)
  {
    this->RecordEndScore();
  }

  this->nextGate = this->gates.begin();
  this->nextGateSide = -1;
  this->runStartTimeWall = common::Time::GetWallTime();
  this->startTimeSim = common::Time::Zero;
  this->startTimeWall = common::Time::Zero;
  this->stopTimeSim = common::Time::Zero;
  this->stopTimeWall = common::Time::Zero;
  this->currentSimTime = common::Time::Zero;
  this->prevScoreTime = common::Time::Zero;
  this->elapsedTimeSim = common::Time::Zero;
  this->username = "guest";
  this->completionScore = 0;
  this->runCompleted = false;

  // reset model pose
  if (this->turtleBot)
    this->turtleBot->SetWorldPose(this->initialPose);

  // start another score recording session
  this->scoreFileStream.open(this->scoreFilePath.string().c_str(),
                             std::fstream::out);
  if (!this->scoreFileStream.is_open())
  {
    gzerr << "Failed to open score file :" << this->scoreFilePath <<
      std::endl;
    return;
  }
}

/////////////////////////////////////////////////
void GateScoringPlugin::StartClock(const common::Time &_simTime,
                                  const common::Time &_wallTime,
                                  std::string &_msg)
{
  this->startTimeSim = _simTime;
  this->startTimeWall = _wallTime;
  std::stringstream ss;
  ss << "Starting clock. ";
  gzlog << ss.str() << std::endl;
  _msg += ss.str();
}

/////////////////////////////////////////////////
void GateScoringPlugin::StopClock(const common::Time &_simTime,
                                 const common::Time &_wallTime,
                                 std::string &_msg)
{
  this->stopTimeSim = _simTime;
  this->stopTimeWall = _wallTime;
  std::stringstream ss;
  ss << "Stopping clock. ";
  gzlog << ss.str() << std::endl;
  _msg += ss.str();
}

/////////////////////////////////////////////////
void GateScoringPlugin::WriteScore(const common::Time &_simTime,
  const common::Time &_wallTime, const std::string &_msg, bool _force)
{
  // Write at 1Hz
  if (!_force && (_simTime - this->prevScoreTime).Double() < 1.0)
    return;

  // If we're being forced, that means that something interesting happened.
  // Also force the gazebo state logger to write.
  if (_force)
  {
     gzdbg << "GateScoringPlugin forcing LogRecord to write" << std::endl;
     util::LogRecord::Instance()->Notify();
  }

  if (!this->scoreFileStream.is_open())
  {
    gzerr << "Score file stream is no longer open:" << this->scoreFilePath <<
      std::endl;
    return;
  }

  // If we've passed the first gate, compute elapsed time
  if (this->stopTimeSim != common::Time::Zero)
    this->elapsedTimeSim = this->stopTimeSim - this->startTimeSim;
  else if (this->startTimeSim != common::Time::Zero)
  {
    this->elapsedTimeSim = _simTime - this->startTimeSim;
  }

  common::Time elapsedTimeWall;
  if (this->stopTimeWall != common::Time::Zero)
    elapsedTimeWall = stopTimeWall - startTimeWall;
  else if (this->startTimeWall != common::Time::Zero)
    elapsedTimeWall = _wallTime - this->startTimeWall;

  common::Time runElapsedTimeWall = _wallTime - this->runStartTimeWall;

  // output message in json format
  this->scoreFileStream << std::fixed << std::setprecision(3)
    << "{"
    << "\"wall_time\": " << runElapsedTimeWall.Double() << ", "
    << "\"sim_time\": " << _simTime.Double() << ", "
    << "\"wall_time_elapsed\": " << elapsedTimeWall.Double() << ", "
    << "\"sim_time_elapsed\": " << elapsedTimeSim.Double() << ", "
    << "\"completion_score\": " << this->completionScore << ", "
    << "\"message\": \"" << _msg << "\""
    << "}" << std::endl;

  // Also publish via ROS

  scoring_plugins::Score rosScoreMsg;
  rosScoreMsg.wall_time = ros::Time(runElapsedTimeWall.Double());
  rosScoreMsg.sim_time = ros::Time(_simTime.Double());
  rosScoreMsg.wall_time_elapsed = ros::Time(elapsedTimeWall.Double());
  rosScoreMsg.sim_time_elapsed = ros::Time(elapsedTimeSim.Double());
  rosScoreMsg.completion_score = this->completionScore;
  
  rosScoreMsg.message = _msg;

  this->pubScoreQueue->push(rosScoreMsg, this->pubScore);

  this->prevScoreTime = _simTime;
}

/////////////////////////////////////////////////
int GateScoringPlugin::IsPoseInGate(const math::Pose& _robotWorldPose,
                                   const math::Pose& _gateWorldPose,
                                   double _gateWidth)
{
  // Transform to gate frame
  math::Vector3 robotLocalPosition =
    _gateWorldPose.rot.GetInverse().RotateVector(_robotWorldPose.pos -
    _gateWorldPose.pos);

  // Are we within the width?
  if (fabs(robotLocalPosition.y) <= _gateWidth / 2.0)
    return (robotLocalPosition.x >= 0.0) ? 1 : -1;
  else
    return 0;
}

/////////////////////////////////////////////////
bool GateScoringPlugin::CheckNextGate(std::string &_msg)
{
  if (!this->turtleBot)
    return false;

  if (this->nextGate != this->gates.end())
  {
    // Get the pose of the robot or the vehicle, depending on the type of the
    // gate.
    math::Pose pose;
    std::string tmpString;
    pose = this->turtleBot->GetWorldPose();
    
    // Figure whether we're positioned before (-1), after (1), or
    // neither (0), with respect to the gate.
    int gateSide = this->IsPoseInGate(pose,
                                      this->nextGate->pose,
                                      this->nextGate->width);
    // Did we go forward through the gate?
    if ((this->nextGateSide < 0) && (gateSide > 0))
    {
      // Log it
      std::stringstream ss;
      ss << "Successfully passed through gate " <<
        (this->nextGate->number+1) << ". ";
      gzlog << ss.str() << std::endl;
      _msg += ss.str();

      // Update state to look for the next gate
      ++this->nextGate;
      this->nextGateSide = 0;
      return true;
    }
    else
    {
      // Just checking: did we go backward through the gate?
      if ((this->nextGateSide > 0) && (gateSide < 0))
      {
        gzlog << "Went backward through gate " <<
          (this->nextGate->number+1) << std::endl;
      }
      // Remember which side we're on now (which might be 0)
      this->nextGateSide = gateSide;
    }
  }
  return false;
}

/////////////////////////////////////////////////
void GateScoringPlugin::OnUpdate(const common::UpdateInfo &_info)
{
//  gzerr << " update " << std::endl;
  if (this->currentSimTime > _info.simTime)
  {
    this->Reset();
  }
  this->currentSimTime = _info.simTime;

  int prevScore = this->completionScore;
  std::string scoreMsg;
  bool forceLogScore = false;

  common::Time simTime = _info.simTime;
  common::Time wallTime = common::Time::GetWallTime();

  // Did we pass through a gate?
  bool firstGate = (this->nextGate == this->gates.begin());
  // We don't count the first gate in the score.
  if (firstGate)
  {
    if (this->CheckNextGate(scoreMsg))
    {
      // first gate passed
      // Force score output so that this event appears in the log
      forceLogScore = true;
      this->StartClock(simTime, wallTime, scoreMsg);
    }
  }
  else
  {
    
    if (this->CheckNextGate(scoreMsg))
    {
      // a new gate has been passed
      this->completionScore += 1;
      // If it's the last gate, we're done
      if (this->nextGate == this->gates.end())
        this->StopClock(simTime, wallTime, scoreMsg);
      
    } 
  }
 
  // Write score data, forcing a write if any score changed;
  // when not forced, it's throttled internally to
  // write at a fixed rate.
  if (prevScore != this->completionScore)
    forceLogScore = true;
  this->WriteScore(simTime, wallTime, scoreMsg, forceLogScore);

  if (this->nextGate == this->gates.end() && !this->runCompleted)
  {
    this->RecordEndScore();
    this->runCompleted = true;
  }
}

/////////////////////////////////////////////////
void GateScoringPlugin::RecordEndScore()
{
  //gzerr << this->scoreFilePath.string().c_str() << std::endl;
  // common::Time timestamp = common::Time::GetWallTime();  
  //long int timestamp = static_cast<long int> time(NULL);
  std::string timestamp = common::Time::GetWallTimeAsISOString();

  std::stringstream ss;
  ss << this->scoreFilePath.leaf().string() << "." << timestamp;
  
  boost::filesystem::path destPath(this->scoreFilePath);
  destPath.remove_leaf() /= ss.str();

  gzerr << destPath.string().c_str() << std::endl;

  boost::filesystem::copy_file(this->scoreFilePath,
    destPath, boost::filesystem::copy_option::overwrite_if_exists);

  this->leaderboardFileStream << std::fixed << std::setprecision(3)
    << "{" 
    << "\"timestamp\": \"" << timestamp << "\", "
    << "\"name\": \"" << this->username << "\", "
    << "\"time\": " << this->elapsedTimeSim.Double() << ", "
    << "\"score\": " << this->completionScore
    << "}" << std::endl;
}

/////////////////////////////////////////////////
bool GateScoringPlugin::FindGates()
{
  // Walk through the world and accumulate the things that appear to be gates.
  physics::Model_V models = this->world->GetModels();
  for (physics::Model_V::const_iterator it = models.begin();
       it != models.end();
       ++it)
  {
    // Parse the name, assuming that gates are named 'gate_<int>'
    physics::ModelPtr model = *it;
    std::string name = model->GetName();
    std::vector<std::string> parts;
    boost::split(parts, name, boost::is_any_of("_"));
    if (parts.size() == 2 && (parts[0] == "gate") )
    {
      // Parse out the number; skip if it fails
      unsigned int gateNum;
      try
      {
        gateNum = boost::lexical_cast<unsigned int>(parts[1]);
      }
      catch (const boost::bad_lexical_cast& e)
      {
        gzwarn << "Ignoring gate name that failed to parse: " << name <<
          std::endl;
        continue;
      }
      // Determine width of gate; it's the larger of the X and Y dimensions of
      // the bounding box of the gate.
      math::Box bbox = model->GetBoundingBox();
      math::Vector3 bboxSize = bbox.GetSize();
      double gateWidth = std::max(bboxSize.x, bboxSize.y);

      Gate::GateType gateType;
      gateType = Gate::PEDESTRIAN;

      // Store this gate
      Gate g(name, gateType, gateNum, model->GetWorldPose(), gateWidth);
      this->gates.push_back(g);
      gzlog << "Stored gate named " << g.name << " of type " << g.type
        << " with index " << g.number << " at pose " << g.pose
        << " and width " << g.width << std::endl;
    }
  }

  if (this->gates.empty())
  {
    gzerr << "Found no gates." << std::endl;
    this->nextGate = this->gates.end();
    return false;
  }

  // Sort in order of increasing gate number (in case we encountered them
  // out-of-order in the list of models).
  this->gates.sort();
  // Set the first gate we're looking for
  this->nextGate = this->gates.begin();
  this->nextGateSide = -1;

  return true;
}

GZ_REGISTER_WORLD_PLUGIN(GateScoringPlugin)


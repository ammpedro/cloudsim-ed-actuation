
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
#ifndef _GAZEBO_GATE_SCORING_PLUGIN_HH_
#define _GAZEBO_GATE_SCORING_PLUGIN_HH_

#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/filesystem.hpp>

#include <gazebo/math/Pose.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <scoring_plugins/Score.h>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  /// \brief A plugin that implements the VRC scoring algorithms.
  class GateScoringPlugin : public WorldPlugin
  {
    /// \brief Constructor
    public: GateScoringPlugin();

    /// \brief Destructor
    public: virtual ~GateScoringPlugin();

    /// \brief Load the plugin
    /// \param[in] _world Pointer to the world.
    /// \param[in] _sdf Point the to world's SDF.
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Called when the world is updated.
    /// \param[in] _info Current world information.
    public: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

    /// \brief Check the next gate to see if we've passed it
    /// \param _msg Log messages (e.g., "passed gate") will be appended here
    /// \return true if the next gate was passed, false otherwise
    private: bool CheckNextGate(std::string &_msg);

    /// \brief Start the clock, used in computing elapsed time for the run
    /// \param _simTime Current simulation time
    /// \param _wallTime Current wallclock time
    /// \param _msg Log messages (e.g., "starting clock") will be appended
    private: void StartClock(const common::Time &_simTime,
                             const common::Time &_wallTime,
                             std::string &_msg);

    /// \brief Stop the clock, used in computing elapsed time for the run
    /// \param _simTime Current simulation time
    /// \param _wallTime Current wallclock time
    /// \param _msg Log messages (e.g., "stopping clock") will be appended
    private: void StopClock(const common::Time &_simTime,
                            const common::Time &_wallTime,
                            std::string &_msg);

    /// \brief Write intermediate score data
    /// \param _simTime Current simulation time
    /// \param _wallTime Current wallclock time
    /// \param _msg Log message to include
    /// \param _force If true, write output; otherwise write output only if
    /// enough time has passed since the last write.
    private: void WriteScore(const gazebo::common::Time& _simTime,
      const common::Time &_wallTime, const std::string &_msg, bool _force);

    /// \brief Find the gates in the world and store them in this->gates.
    private: bool FindGates();

     /// \brief Is the given robot pose "in" the given gate pose?
    /// \param _robotWorldPose Pose of the robot, in the world frame
    /// \param _gateWorldPose Pose of the gate, in the world frame
    /// \param _gateWorldPose Width of the gate
    /// \return If not "in" the gate, return 0; else return -1 if "before" the
    ///         gate, 1 if "after" the gate.
    private: int IsPoseInGate(const gazebo::math::Pose& _robotWorldPose,
                              const gazebo::math::Pose& _gateWorldPose,
                              double _gateWidth);

    /// \brief Reset the plugin
    private: void Reset();

    /// \brief Save a copy of score to file with timestamp.
    private: void RecordEndScore();

    /// \brief Data about a gate.
    private: class Gate
             {
               /// \brief Types of gates that we know about
               public: enum GateType
               {
                 PEDESTRIAN,
                 VEHICLE
               };

               public: Gate(const std::string &_name,
                            GateType _type,
                            unsigned int _number,
                            const gazebo::math::Pose& _pose,
                            double _width)
                         : name(_name), type(_type),
                           number(_number), pose(_pose),
                           width(_width), passed(false) {}

               /// \brief Less-than operator to allow sorting of a list of
               /// gates by number.
               public: bool operator< (Gate &other)
                       {
                         return (this->number < other.number);
                       }

               /// \brief Name of the gate
               public: std::string name;

               /// \brief The type of the gate
               public: GateType type;

               /// \brief Number of the gate
               public: unsigned int number;

               /// \brief Pose of the center of the gate
               public: gazebo::math::Pose pose;

               /// \brief Width of the gate
               public: double width;

               /// \brief Have we passed through this gate yet?
               public: bool passed;
             };

    /// \brief The worlds that we might be scoring; each one can be
    /// slightly different
    private: enum WorldType
             {
               QUAL_1,
               QUAL_2,
               QUAL_3,
               QUAL_4,
               VRC_1,
               VRC_2,
               VRC_3
             };

    /// \brief Pointer to the world.
    private: physics::WorldPtr world;

    /// \brief Pointer to TurtleBot.
    private: physics::ModelPtr turtleBot;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief List of all the gates in the world. We assume that gates have
    /// the names: gate_1, gate_2, ..., gate_n.
    private: std::list<Gate> gates;

    /// \brief Which gate is expected next, expressed as an iterator into
    /// this->gates.
    private: std::list<Gate>::iterator nextGate;

    /// \brief Which side of the next gate we were the last time we checked.
    private: int nextGateSide;

    /// \brief The absolute wall time when the run started
    private: common::Time runStartTimeWall;

    /// \brief Sim time at which TurtleBot passed through the first gate.
    private: gazebo::common::Time startTimeSim;

    /// \brief Wall time at which TurtleBot passed through the first gate.
    private: gazebo::common::Time startTimeWall;

    /// \brief Sim time at which TurtleBot achieved the last checkpoint.
    private: gazebo::common::Time stopTimeSim;

    /// \brief Wall time at which TurtleBot achieved the last checkpoint.
    private: gazebo::common::Time stopTimeWall;

    /// \brief Current sim time used to track whether or not there was a reset.
    private: gazebo::common::Time currentSimTime;

    /// \brief Initial pose of model;
    private: gazebo::math::Pose initialPose;

    /// \brief True if the model passed all gates;
    private: bool runCompleted;

    /// \brief The completion score, called 'C' in the VRC docs
    private: int completionScore;

    /// \brief Name of the file that we're writing score data to
    private: boost::filesystem::path scoreFilePath;

    /// \brief Name of the file that we're writing all score data to
    private: boost::filesystem::path leaderboardFilePath;

    /// \brief The stream associated with scoreFilePath
    private: std::ofstream scoreFileStream;

    /// \brief The stream associated with leaderboardFilePath
    private: std::ofstream leaderboardFileStream;

    /// \brief Sim time elapsed since crossing the first gate
    private: common::Time elapsedTimeSim;

    /// \brief User name.
    private: std::string username;

    /// \brief When we last wrote score data to disk
    private: common::Time prevScoreTime;

    /// \brief Which type of world we're scoring
    private: enum WorldType worldType;

    /// \brief ros node handle
    private: ros::NodeHandle *rosNode;

    /// \brief publisher of vrc_score
    private: ros::Publisher pubScore;
    private: PubQueue<scoring_plugins::Score>::Ptr pubScoreQueue;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue* pmq;
    private: boost::thread deferredLoadThread;

    // \brief Elapsed sim time after task completion when we stop counting
    // falls.  It's non-zero to avoid having people dive across the finish
    // line.
    private: const common::Time postCompletionQuietTime;
  };
}
#endif


/****************************************************************
 *
 * Copyright (c) 2016
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Simple class for downsampling of grid-based planner
 *  trajectory (in list of waypoints format) and incorporating the 
 *  final pose information in the final waypoint
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Jan Filip, email:jan.filip@esa.int, jan.filip2@gmail.com
 * Supervised by: Martin Azkarate, email:martin.azkarate@esa.int
 *
 * Date of creation: Oct 2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */


#ifndef TRAJECTORY_HELPERS_TRAJECTORYREFINER_TASK_HPP
#define TRAJECTORY_HELPERS_TRAJECTORYREFINER_TASK_HPP

#include "trajectory_helpers/TrajectoryRefinerBase.hpp"

namespace trajectory_helpers {

    class TrajectoryRefiner : public TrajectoryRefinerBase
    {
	friend class TrajectoryRefinerBase;
    protected:
        std::vector<base::Waypoint> waypointsIn;
        std::vector<base::Waypoint> waypointsOut;
        base::samples::RigidBodyState goalPose;
        bool goalPoseSet;
    public:
        
        TrajectoryRefiner(std::string const& name = "trajectory_helpers::TrajectoryRefiner");

        TrajectoryRefiner(std::string const& name, RTT::ExecutionEngine* engine);

        
	~TrajectoryRefiner();

        bool configureHook();
       
        bool startHook();
     
        void updateHook();
      
        void errorHook();
        
        void stopHook();

        void cleanupHook();

    private:
        bool preprocessPath();
        void addGoal();
        void printStatus();
    };
}

#endif


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

#include "TrajectoryRefiner.hpp"

using namespace trajectory_helpers;

TrajectoryRefiner::TrajectoryRefiner(std::string const& name)
    : TrajectoryRefinerBase(name), waypointsIn(), waypointsOut()
{
}

TrajectoryRefiner::TrajectoryRefiner(std::string const& name, RTT::ExecutionEngine* engine)
    : TrajectoryRefinerBase(name, engine), waypointsIn(), waypointsOut()
{
}

TrajectoryRefiner::~TrajectoryRefiner()
{
}

bool TrajectoryRefiner::configureHook()
{
    if (! TrajectoryRefinerBase::configureHook())
        return false;
    goalPoseSet = false;
    goal_position_tol = _goal_position_tol.value() > 0 ? _goal_position_tol.value() : 0.05;
    goal_heading_tol  = _goal_heading_tol.value()  > 0 ? _goal_heading_tol.value()/180.0*M_PI : 2.0/180.0*M_PI;
    std::cout << "TrajectoryRefiner::configureHook() default tolerance " 
    << goal_position_tol << "m, "
    << goal_heading_tol*180.0/M_PI << "deg." << std::endl;
    return true;
}
bool TrajectoryRefiner::startHook()
{
    if (! TrajectoryRefinerBase::startHook())
        return false;
    return true;
}
void TrajectoryRefiner::updateHook()
{
    TrajectoryRefinerBase::updateHook();
    int rtt_return = _goal_pose.readNewest(goalPose);
    if(rtt_return  == RTT::NewData && rtt_return  != RTT::NoData) {
        goalPoseSet = true;
    }

    rtt_return = _waypoints_in.readNewest(waypointsIn);  
    if(rtt_return  == RTT::NewData ) { // Trajectory input contains new data
        if (preprocessPath() & goalPoseSet){
            addGoal();
        }
        printStatus();
        _waypoints_out.write(waypointsOut);
    }
}

void TrajectoryRefiner::errorHook()
{
    TrajectoryRefinerBase::errorHook();
}
void TrajectoryRefiner::stopHook()
{
    TrajectoryRefinerBase::stopHook();
}
void TrajectoryRefiner::cleanupHook()
{
    TrajectoryRefinerBase::cleanupHook();
    waypointsIn.clear();
    waypointsOut.clear();
}

void TrajectoryRefiner::addGoal(){
    waypointsOut.back().heading      = goalPose.getYaw();
    waypointsOut.back().tol_position = goal_position_tol;
    waypointsOut.back().tol_heading  = goal_heading_tol;
}

bool TrajectoryRefiner::preprocessPath(){
    // Downsampling requested/
    waypointsOut.clear();       
    if (waypointsIn.size() < 3) {
                // 0, 1, or 2 waypoints pass with no change
        waypointsOut = waypointsIn;
    } else {
        int dxPrev, dyPrev, dxNext, dyNext;
        int unitScale = 100; // to cm
        base::Vector3d currWp, nextWp;
        
        // Initialize //
        currWp = waypointsIn.at(0).position;
        nextWp = waypointsIn.at(1).position;
        dxPrev = round(unitScale*(nextWp.x() - currWp.x()));
        dyPrev = round(unitScale*(nextWp.y() - currWp.y()));
        waypointsOut.push_back(waypointsIn.front()); 

        // Iterate //
        for (size_t it = 1; it < waypointsIn.size() - 1 ; it++) 
        {
            // Shift the waypoints being examined //
            currWp = nextWp;
            nextWp = waypointsIn.at(it+1).position;

            // Get the [dx, dy] vector to the next //
            dxNext = round(unitScale*(nextWp.x() - currWp.x()));
            dyNext = round(unitScale*(nextWp.y() - currWp.y()));

            // Is the waypoint significant or lies on a line defined by the prev and next?
            if (dxPrev != dxNext || dyPrev != dyNext){
                // Significant, will be added
                waypointsOut.push_back(waypointsIn.at(it)); 
                // Otherwise does not have to be on the path.
            }
            // Shift the calculated [dx, dy] to be used in next iteration //
            dxPrev = dxNext;
            dyPrev = dyNext;
        }
        // Terminate //
        waypointsOut.push_back(waypointsIn.back());
    }
    return (waypointsOut.size() > 0);
}

void TrajectoryRefiner::printStatus(){
    std::cout << "TrajectoryRefiner received \t"    << waypointsIn.size()   << " pts."
        << std::endl 
            << "TrajectoryRefiner downsampled \t"   << waypointsOut.size() << " pts."
        << std::endl;
    if (waypointsOut.size()>0) {
        std::cout << "TrajectoryRefiner final heading set to \t" << waypointsOut.back().heading*180.0/M_PI
                  << " +- " << waypointsOut.back().tol_heading*180.0/M_PI << std::endl;
        }
}


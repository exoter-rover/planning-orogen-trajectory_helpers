/****************************************************************
 *
 * Copyright (c) 2016
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Class for goal pose generation from a 2D position 
 *  and heading information provided in a .yaml config file
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

#ifndef TRAVERSABILITY_EXPLORER_GOALGENERATOR_TASK_HPP
#define TRAVERSABILITY_EXPLORER_GOALGENERATOR_TASK_HPP

#include "trajectory_helpers/GoalGeneratorBase.hpp"

namespace trajectory_helpers {
    class GoalGenerator : public GoalGeneratorBase
    {
	friend class GoalGeneratorBase;
    protected:
        base::samples::RigidBodyState goalPose;


    public:
        GoalGenerator(std::string const& name = "traversability_explorer::GoalGenerator");
        GoalGenerator(std::string const& name, RTT::ExecutionEngine* engine);
	   ~GoalGenerator();
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif


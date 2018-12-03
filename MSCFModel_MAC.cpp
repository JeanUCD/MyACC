/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    MSCFModel_MAC.cpp
/// @author  Kallirroi Porfyri
/// @date    Feb 2018
/// @version $Id$
///
// ACC car-following model based on [1], [2].
// [1] Milanes, V., and S. E. Shladover. Handling Cut-In Vehicles in Strings
//    of Cooperative Adaptive Cruise Control Vehicles. Journal of Intelligent
//     Transportation Systems, Vol. 20, No. 2, 2015, pp. 178-191.
// [2] Xiao, L., M. Wang and B. van Arem. Realistic Car-Following Models for
//    Microscopic Simulation of Adaptive and Cooperative Adaptive Cruise
//     Control Vehicles. Transportation Research Record: Journal of the
//     Transportation Research Board, No. 2623, 2017. (DOI: 10.3141/2623-01).
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <stdio.h>
#include <iostream>

#include "MSCFModel_MAC.h"
#include <microsim/MSVehicle.h>
#include <microsim/MSLane.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOTime.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <math.h>
#include <microsim/MSNet.h>

// ===========================================================================
// debug flags
// ===========================================================================
#define DEBUG_MAC
#define DEBUG_COND (veh->isSelected())


// ===========================================================================
// defaults
// ===========================================================================
#define DEFAULT_SC_GAIN -0.4
#define DEFAULT_GCC_GAIN_SPEED 0.8
#define DEFAULT_GCC_GAIN_SPACE 0.04
#define DEFAULT_GC_GAIN_SPEED 0.07
#define DEFAULT_GC_GAIN_SPACE 0.23
#define DEFAULT_CA_GAIN_SPACE 0.8
#define DEFAULT_CA_GAIN_SPEED 0.23

// override followSpeed when deemed unsafe by the given margin (the value was selected to reduce the number of necessary interventions)
#define DEFAULT_EMERGENCY_OVERRIDE_THRESHOLD 2.0

/// @todo: add attributes for myCollisionAvoidanceGainSpeed and myCollisionAvoidanceGainSpace

// ===========================================================================
// method definitions
// ===========================================================================
MSCFModel_MAC::MSCFModel_MAC(const MSVehicleType* vtype) :
    MSCFModel(vtype),
    mySpeedControlGain(vtype->getParameter().getCFParam(SUMO_ATTR_SC_GAIN, DEFAULT_SC_GAIN)), //k1
    myGapClosingControlGainSpeed(vtype->getParameter().getCFParam(SUMO_ATTR_GCC_GAIN_SPEED, DEFAULT_GCC_GAIN_SPEED)),
    myGapClosingControlGainSpace(vtype->getParameter().getCFParam(SUMO_ATTR_GCC_GAIN_SPACE, DEFAULT_GCC_GAIN_SPACE)),
    myGapControlGainSpeed(vtype->getParameter().getCFParam(SUMO_ATTR_GC_GAIN_SPEED, DEFAULT_GC_GAIN_SPEED)),
    myGapControlGainSpace(vtype->getParameter().getCFParam(SUMO_ATTR_GC_GAIN_SPACE, DEFAULT_GC_GAIN_SPACE)),
    myCollisionAvoidanceGainSpeed(vtype->getParameter().getCFParam(SUMO_ATTR_CA_GAIN_SPEED, DEFAULT_CA_GAIN_SPEED)),
    myCollisionAvoidanceGainSpace(vtype->getParameter().getCFParam(SUMO_ATTR_CA_GAIN_SPACE, DEFAULT_CA_GAIN_SPACE)) {
    // ACC does not drive very precise and often violates minGap
    myCollisionMinGapFactor = vtype->getParameter().getCFParam(SUMO_ATTR_COLLISION_MINGAP_FACTOR, 0.1);
}

MSCFModel_MAC::~MSCFModel_MAC() {}


SUMOReal
MSCFModel_MAC::followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap2pred, SUMOReal predSpeed, SUMOReal predMaxDecel, const MSVehicle* const /*pred*/) const {
    const SUMOReal desSpeed = MIN2(veh->getLane()->getSpeedLimit(), veh->getMaxSpeed()); // 车辆最大速度
    const SUMOReal vACC = _v(veh, gap2pred, speed, predSpeed, desSpeed, true); // vACC: 最大跟驰速度？
    const SUMOReal vSafe = maximumSafeFollowSpeed(gap2pred, speed, predSpeed, predMaxDecel);// 安全跟驰速度？
    if (vSafe + DEFAULT_EMERGENCY_OVERRIDE_THRESHOLD < vACC) {
        //ACCVehicleVariables* vars = (ACCVehicleVariables*)veh->getCarFollowVariables();
        //std::cout << SIMTIME << " veh=" << veh->getID() << " v=" << speed << " vL=" << predSpeed << " gap=" << gap2pred << " vACC=" << vACC << " vSafe=" << vSafe << " cm=" << vars->ACC_ControlMode << "\n";
        return vSafe + DEFAULT_EMERGENCY_OVERRIDE_THRESHOLD;
    }
    return vACC;
}


SUMOReal
MSCFModel_MAC::stopSpeed(const MSVehicle* const veh, const SUMOReal speed, SUMOReal gap) const {
    // NOTE: This allows return of smaller values than minNextSpeed().
    // Only relevant for the ballistic update: We give the argument headway=TS, to assure that
    // the stopping position is approached with a uniform deceleration also for tau!=TS.
    return MIN2(maximumSafeStopSpeed(gap, speed, false, veh->getActionStepLengthSecs()), maxNextSpeed(speed, veh));
}


/// @todo update interactionGap logic
SUMOReal
MSCFModel_MAC::interactionGap(const MSVehicle* const /* veh */, SUMOReal /* vL */) const { //vL: 前车速度
    /*maximum radar range is ACC is enabled*/
    return 250; // 两车距离小于250，前车速度影响后车速度
}

SUMOReal MSCFModel_MAC::accelSpeedControl(SUMOReal vErr) const { // vErr: 当前速度与最大速度的差
    // Speed control law
    return mySpeedControlGain * vErr;
}

SUMOReal MSCFModel_MAC::accelGapControl(const MSVehicle* const veh, const SUMOReal gap2pred, const SUMOReal speed, const SUMOReal predSpeed) const {

#ifdef DEBUG_ACC
    if DEBUG_COND {
    std::cout << "        applying gapControl" << std::endl;
}
#endif

// Gap control law
SUMOReal gclAccel = 0.0;
SUMOReal desSpacing = myHeadwayTime * speed; // 反应时间的制动距离？
// The argument gap2pred does not consider minGap ->  substract minGap!!
// XXX: It does! (Leo)
SUMOReal gap = gap2pred - veh->getVehicleType().getMinGap();
    SUMOReal spacingErr = gap - desSpacing;
    SUMOReal deltaVel = predSpeed - speed;
    if (fabs(spacingErr) < 0.2 && fabs(deltaVel) < 0.1) {
        // gap mode
        gclAccel = myGapControlGainSpeed * deltaVel + myGapControlGainSpace * spacingErr;
    } else if (spacingErr < 0)  {
        // collision avoidance mode
        gclAccel = myCollisionAvoidanceGainSpeed * deltaVel + myCollisionAvoidanceGainSpace * spacingErr;
    } else {
        // gap closing mode
        gclAccel = myGapClosingControlGainSpeed * deltaVel + myGapClosingControlGainSpace * spacingErr;
    }

    return gclAccel;
}


SUMOReal
MSCFModel_MAC::_v(const MSVehicle* const veh, const SUMOReal gap2pred, const SUMOReal speed,
                  const SUMOReal predSpeed, const SUMOReal desSpeed, const bool /* respectMinGap */) const {

    SUMOReal accelACC = 0;
    SUMOReal gapLimit_SC = 120; // lower gap limit in meters to enable speed control law
    SUMOReal gapLimit_GC = 100; // upper gap limit in meters to enable gap control law

#ifdef DEBUG_ACC
    if DEBUG_COND {
    std::cout << SIMTIME << " MSCFModel_MAC::_v() for veh '" << veh->getID() << "'\n"
        << "        gap=" << gap2pred << " speed="  << speed << " predSpeed=" << predSpeed
        << " desSpeed=" << desSpeed << std::endl;
    }
#endif


    /* Velocity error */
    SUMOReal vErr = speed - desSpeed;
    int setControlMode = 0;
    ACCVehicleVariables* vars = (ACCVehicleVariables*)veh->getCarFollowVariables();
    if (vars->lastUpdateTime != MSNet::getInstance()->getCurrentTimeStep()) {
        vars->lastUpdateTime = MSNet::getInstance()->getCurrentTimeStep();
        setControlMode = 1;
    }
    if (gap2pred > gapLimit_SC) {

#ifdef DEBUG_ACC
        if DEBUG_COND {
        std::cout << "        applying speedControl" << std::endl;
    }
#endif
    // Find acceleration - Speed control law
    accelACC = accelSpeedControl(vErr);
        // Set cl to vehicle parameters
        if (setControlMode) {
            vars->ACC_ControlMode = 0;
        }
    } else if (gap2pred < gapLimit_GC) {
        // Find acceleration - Gap control law
        accelACC = accelGapControl(veh, gap2pred, speed, predSpeed, vErr);
        // Set cl to vehicle parameters
        if (setControlMode) {
            vars->ACC_ControlMode = 1;
        }
    } else {
        // Follow previous applied law
        int cm = vars->ACC_ControlMode;
        if (!cm) {

#ifdef DEBUG_ACC
            if DEBUG_COND {
            std::cout << "        applying speedControl" << std::endl;
        }
#endif
        accelACC = accelSpeedControl(vErr);
        } else {
            accelACC = accelGapControl(veh, gap2pred, speed, predSpeed, vErr);
        }

    }

    SUMOReal newSpeed = speed + ACCEL2SPEED(accelACC);

#ifdef DEBUG_ACC
    if DEBUG_COND {
    std::cout << "        result: accel=" << accelACC << " newSpeed="  << newSpeed << std::endl;
}
#endif

return MAX2(0., newSpeed);
}


MSCFModel*
MSCFModel_MAC::duplicate(const MSVehicleType* vtype) const {
    return new MSCFModel_MAC(vtype);
}

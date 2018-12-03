#/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    MSCFModel_ACC.h
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
#ifndef MSCFModel_MAC_H
#define MSCFModel_MAC_H

// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include "MSCFModel.h"
#include <utils/xml/SUMOXMLDefinitions.h>

// ===========================================================================
// class declarations
// ===========================================================================
class MSVehicle;
class MSVehicleType;
class MSCFModel_CACC;

// ===========================================================================
// class definitions
// ===========================================================================
/** @class MSCFModel_MAC
* @brief The MAC car-following model
* @see MSCFModel
*/
class MSCFModel_MAC : public MSCFModel {
public:
    /** @brief Constructor
     *  @param[in] vtype the type for which this model is built and also the parameter object to configure this model
     */
    MSCFModel_MAC(const MSVehicleType* vtype);

    /// @brief Destructor
    ~MSCFModel_MAC();


    /// @name Implementations of the MSCFModel interface
    /// @{

    /** @brief Computes the vehicle's safe speed (no dawdling)
    * @param[in] veh The vehicle (EGO)
    * @param[in] speed The vehicle's speed
    * @param[in] gap2pred The (netto) distance to the LEADER
    * @param[in] predSpeed The speed of LEADER
    * @return EGO's safe speed
    * @see MSCFModel::ffeV
    */
    SUMOReal followSpeed(const MSVehicle* const veh, SUMOReal speed, SUMOReal gap2pred, SUMOReal predSpeed, SUMOReal predMaxDecel, const MSVehicle* const pred = 0) const;


    /** @brief Computes the vehicle's safe speed for approaching a non-moving obstacle (no dawdling)
    * @param[in] veh The vehicle (EGO)
    * @param[in] gap2pred The (netto) distance to the the obstacle
    * @return EGO's safe speed for approaching a non-moving obstacle
    * @see MSCFModel::ffeS
    * @todo generic Interface, models can call for the values they need
    */
    SUMOReal stopSpeed(const MSVehicle* const veh, const SUMOReal speed, SUMOReal gap2pred) const;


    /** @brief Returns the maximum gap at which an interaction between both vehicles occurs
    *
    * "interaction" means that the LEADER influences EGO's speed.
    * @param[in] veh The EGO vehicle
    * @param[in] vL LEADER's speed
    * @return The interaction gap
    * @todo evaluate signature
    * @see MSCFModel::interactionGap
    */
    SUMOReal interactionGap(const MSVehicle* const, SUMOReal vL) const;


    /** @brief Returns the model's name
    * @return The model's name
    * @see MSCFModel::getModelName
    */
    int getModelID() const {
        return SUMO_TAG_CF_ACC;
    }
    /// @}



    /** @brief Duplicates the car-following model
    * @param[in] vtype The vehicle type this model belongs to (1:1)
    * @return A duplicate of this car-following model
    */
    MSCFModel* duplicate(const MSVehicleType* vtype) const;

    virtual MSCFModel::VehicleVariables* createVehicleVariables() const {
        ACCVehicleVariables* ret = new ACCVehicleVariables();
        ret->ACC_ControlMode = 0;
        ret->lastUpdateTime = 0;
        return ret;
    }

    friend class MSCFModel_CACC;

private:
    class ACCVehicleVariables : public MSCFModel::VehicleVariables {
    public:
        ACCVehicleVariables() : ACC_ControlMode(0) {}
        /// @brief The vehicle's ACC control mode. 0 for speed control and 1 for gap control
        int ACC_ControlMode;
        SUMOTime lastUpdateTime;
    };


private:
    SUMOReal _v(const MSVehicle* const veh, const SUMOReal gap2pred, const SUMOReal mySpeed,
              const SUMOReal predSpeed, const SUMOReal desSpeed, const bool respectMinGap = true) const;

    SUMOReal accelSpeedControl(SUMOReal vErr) const;
    SUMOReal accelGapControl(const MSVehicle* const veh, const SUMOReal gap2pred, const SUMOReal speed, const SUMOReal predSpeed) const;


private:
    SUMOReal mySpeedControlGain;
    SUMOReal myGapClosingControlGainSpeed;
    SUMOReal myGapClosingControlGainSpace;
    SUMOReal myGapControlGainSpeed;
    SUMOReal myGapControlGainSpace;
    SUMOReal myCollisionAvoidanceGainSpeed;
    SUMOReal myCollisionAvoidanceGainSpace;

private:
    /// @brief Invalidated assignment operator
    MSCFModel_MAC& operator=(const MSCFModel_MAC& s);
};

#endif /* MSCFModel_ACC_H */
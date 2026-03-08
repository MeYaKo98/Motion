#pragma once

#include "Motion/Core/Robot/Position.h"
#include "Motion/Core/Robot/Wheel.h"

/*--------------Odom--------------*/
#include "Motion/Core/Robot/Odom/BaseOdometry.h"
#include "Motion/Core/Robot/Odom/GenericDifferentialDriveOdometry.h"
#include "Motion/Core/Robot/Odom/DifferentialDriveOdometry.h"

/*--------------Controller--------------*/
#include "Motion/Core/Robot/Controller/BaseController.h"
#include "Motion/Core/Robot/Controller/PIDController.h"

/*--------------ProfileGenerator--------------*/
#include "Motion/Core/Robot/ProfileGenerator/BaseProfileGenerator.h"
#include "Motion/Core/Robot/ProfileGenerator/StepProfileGenerator.h"
#include "Motion/Core/Robot/ProfileGenerator/TrapezoidalProfileGenerator.h"

/*--------------StopCondition--------------*/
#include "Motion/Core/Robot/StopCondition/BaseStopCondition.h"
#include "Motion/Core/Robot/StopCondition/AndStopCondition.h"
#include "Motion/Core/Robot/StopCondition/OrStopCondition.h"
#include "Motion/Core/Robot/StopCondition/ToleranceStopCondition.h"

/*--------------Navigation--------------*/
#include "Motion/Core/Robot/Navigation/BaseNavigation.h"
#include "Motion/Core/Robot/Navigation/GenericDifferentialDriveNavigation.h"
#include "Motion/Core/Robot/Navigation/DifferentialDriveNavigation.h"
/**
 * @file GenericDifferentialDriveNavigation.h
 * @brief Generic navigation implementation for differential drive robots.
 * @details Provides common navigation logic and state for differential drive systems.
 *          Serves as a base for concrete implementations (e.g., DifferentialDriveNavigation).
 */

#pragma once

#include "Motion/Core/Robot/Navigation/BaseNavigation.h"
#include "Motion/Core/Robot/Odom/GenericDifferentialDriveOdometry.h"
#include "Motion/Core/Robot/Controller/BaseController.h"
#include "Motion/Core/IO/Actuator/GenericMotor.h"

namespace Motion::Core::Robot {

/**
 * @brief Configuration structure for differential drive motors and their controllers.
 * @details Holds pointers to the motors (actuators) and their corresponding feedback controllers.
 *          This configuration is injected into the navigation system to define the motion execution strategy.
 *
 *          **Architecture:**
 *          Each wheel has two components:
 *          - Motor (GenericMotor): Accepts speed commands from the navigation system
 *          - Controller (BaseController): Takes position/velocity error and generates motor commands
 *
 *          **Data Flow:**
 *          Navigation system → computes error → Controller → generates motor command → Motor
 *          Motor → produces motion → Odometry → reads position → closes the loop
 *
 * @note **Separation of Concerns:** Controllers handle feedback control; motors handle actuation.
 *       This allows using the same controller with different motor types.
 */
struct DifferentialDriveMotorConfig {
    /**
     * @brief Smart pointer to the right wheel's motor (IO::GenericMotor).
     * @details Used to command the right motor during navigation operations.
     *          Receives speed commands (typically [-1.0, 1.0] normalized).
     * @note Must not be nullptr; factory validates this.
     * @note Ownership: Caller retains ownership; navigation holds a reference.
     */
    Motion::Core::IO::GenericMotorHandle rightMotorHandle;

    /**
     * @brief Smart pointer to the left wheel's motor (IO::GenericMotor).
     * @details Used to command the left motor during navigation operations.
     *          Receives speed commands (typically [-1.0, 1.0] normalized).
     * @note Must not be nullptr; factory validates this.
     */
    Motion::Core::IO::GenericMotorHandle leftMotorHandle;

    /**
     * @brief Smart pointer to the right motor's feedback controller (BaseController).
     * @details Typically a PIDController that converts position/velocity error into motor commands.
     *          Closed-loop control ensures the motor maintains desired speed despite load variations.
     *
     * @note Must not be nullptr; factory validates this.
     * @note Each motor should have its own controller instance for independent tuning.
     */
    BaseControllerHandle rightControllerHandle;

    /**
     * @brief Smart pointer to the left motor's feedback controller (BaseController).
     * @details Typically a PIDController that converts position/velocity error into motor commands.
     *          Allows independent speed control of left and right wheels for turning.
     * @note Must not be nullptr; factory validates this.
     */
    BaseControllerHandle leftControllerHandle;
};

/**
 * @brief Generic (base) navigation implementation for differential drive robots.
 * @details This class centralizes common navigation components and state shared by all
 *          differential drive navigation systems. It inherits from BaseNavigation (abstract interface)
 *          and adds differential drive-specific state management.
 *
 *          **Components Managed:**
 *          - Odometry: Position tracking from wheel encoders
 *          - Motor Configuration: Left and right wheels with controllers
 *          - Motion Profile Generator: Smooth velocity commands (inherited from BaseNavigation)
 *          - Stop Condition: Motion completion detection (inherited from BaseNavigation)
 *
 *          **Design Strategy:**
 *          - GenericDifferentialDriveNavigation: Common state and components
 *          - DifferentialDriveNavigation: Concrete algorithm implementation (Move, Turn, MoveTo, Orient)
 *          This separation allows reusing common infrastructure across different implementations.
 *
 *          **Inheritance Hierarchy:**
 *          @code
 *          BaseNavigation (abstract interface)
 *              ↑
 *          GenericDifferentialDriveNavigation (common differential drive logic)
 *              ↑
 *          DifferentialDriveNavigation (concrete implementation with motion algorithms)
 *          @endcode
 *
 * @note **Protected Constructor:** GenericDifferentialDriveNavigation has a protected constructor,
 *       indicating it's meant to be a base class, not directly instantiated. Use DifferentialDriveNavigation
 *       via its Create() factory.
 *
 * @note **State Access:** Derived classes can access:
 *       - _odometryHandle: To read robot position
 *       - _motorConfig: To command motors and controllers
 *       - _profileGeneratorHandle (inherited): To generate motion profiles
 *       - _stopConditionHandle (inherited): To detect motion completion
 *
 * @see BaseNavigation for the abstract motion command interface
 * @see DifferentialDriveNavigation for concrete motion algorithm implementation
 * @see DifferentialDriveMotorConfig for motor/controller configuration
 * @see GenericDifferentialDriveOdometry for position tracking
 */
class GenericDifferentialDriveNavigation : public BaseNavigation{
public:
    /**
     * @brief Virtual destructor for polymorphic cleanup.
     * @details Ensures derived class destructors are called properly when deleting
     *          through the base class pointer. Uses default implementation.
     *
     * @note **Default Implementation:** Uses `= default`, which delegates to the
     *       compiler-generated destructor. This is appropriate because this class
     *       doesn't allocate resources directly (holds smart pointers managed by caller).
     */
    virtual ~GenericDifferentialDriveNavigation() = default;

protected:
    /**
     * @brief Constructs a new GenericDifferentialDriveNavigation instance.
     * @details Protected constructor; use DifferentialDriveNavigation::Create() instead.
     *          Initializes all navigation components from the provided handles.
     *
     *          **Component Initialization:**
     *          - _odometryHandle: Stores the odometry system for position queries
     *          - _motorConfig: Stores motor and controller references for motion execution
     *          - BaseNavigation constructor: Initializes profile generator and stop condition
     *
     * @param odometryHandle Smart pointer to the differential drive odometry system.
     *                       Used by motion algorithms to track position and detect motion completion.
     *                       Must not be nullptr (factory validates).
     *
     * @param profileGeneratorHandle Smart pointer to the motion profile generator.
     *                               Generates smooth velocity commands respecting acceleration limits.
     *                               Passed to BaseNavigation constructor.
     *                               Must not be nullptr (factory validates).
     *
     * @param stopConditionHandle Smart pointer to the stop condition evaluator.
     *                            Determines when motion is complete.
     *                            Passed to BaseNavigation constructor.
     *                            Must not be nullptr (factory validates).
     *
     * @param motorConfig Structure containing motor and controller handles for both wheels.
     *                   All four handles must be non-null (factory validates).
     *
     * @post All state is initialized and ready for use.
     * @post Derived classes can access state via protected members.
     *
     * @note **Dependency Injection:** All dependencies are provided as parameters.
     *       This promotes testability and flexibility (can swap implementations).
     *
     * @note **Ownership Model:** The constructor takes references (smart pointers) to dependencies
     *       but does not take ownership. The caller (factory) retains ultimate responsibility
     *       for cleanup. Smart pointers handle automatic cleanup when the last reference is released.
     *
     * @warning **Null Checks:** While the factory validates inputs, this constructor
     *          assumes they're non-null. Do not call this constructor directly with null pointers.
     */
    explicit GenericDifferentialDriveNavigation(
        GenericDifferentialDriveOdometryHandle odometryHandle,
        BaseProfileGeneratorHandle profileGeneratorHandle,
        BaseStopConditionHandle stopConditionHandle,
        DifferentialDriveMotorConfig motorConfig)
        : _odometryHandle(odometryHandle), _motorConfig(motorConfig), BaseNavigation(profileGeneratorHandle, stopConditionHandle) {}

    /**
     * @brief Smart pointer to the differential drive odometry system.
     * @details Used by motion algorithms to:
     *          - Read current robot position (GetPosition())
     *          - Correct odometry drift if needed (SetPosition())
     *          - Determine when motion is complete by comparing current to target position
     *
     * @note **Thread-Safe Access:** The odometry system's GetPosition() method is thread-safe.
     *       Safe to call from any FreeRTOS task.
     *
     * @note **Ownership:** Owned by the caller (typically the application's main setup).
     *       The navigation system holds a smart pointer reference.
     */
    GenericDifferentialDriveOdometryHandle _odometryHandle;

    /**
     * @brief Configuration object holding motor and controller smart pointers.
     * @details Provides access to:
     *          - rightMotorHandle: To command right wheel speed
     *          - leftMotorHandle: To command left wheel speed
     *          - rightControllerHandle: To generate control commands based on right wheel error
     *          - leftControllerHandle: To generate control commands based on left wheel error
     *
     *          **Typical Usage in Derived Class:**
     *          @code
     *          float rightCommand = _motorConfig.rightControllerHandle->GenerateCommand(rightReference, rightReading);
     *          _motorConfig.rightMotorHandle->SetCommand(rightCommand);
     *          @endcode
     *
     * @note **Immutable Structure:** The config is set at construction and should not be modified.
     *       If motor configuration changes, create a new navigation instance.
     *
     * @note **Ownership:** All handles within the config are owned by the caller.
     *       The navigation system holds references.
     *
     * @see DifferentialDriveMotorConfig for structure details
     */
    DifferentialDriveMotorConfig _motorConfig;
};

using GenericDifferentialDriveNavigationHandle = NavigationPointer(GenericDifferentialDriveNavigation);

} // namespace Motion::Core::Robot
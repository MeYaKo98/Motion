/**
 * @file CompleteDifferentialDriveMotionLink.cpp
 * @brief Complete ESP32 differential-drive Motion Link composition template.
 *
 * This example is adapted from the Motion Framework reference application. It
 * shows how the hardware layer, odometry, controllers, profile generator,
 * stop condition, navigation, Wi-Fi, logging, and Motion Link fit together.
 *
 * Replace the pin assignments, calibration values, and Wi-Fi configuration
 * before using this template on a real robot. Keep credentials out of source
 * control; the placeholders below are intentionally not real credentials.
 */

#include <Motion/Core.h>
#include <Motion/Link.h>
#include <WiFi.h>

using namespace Motion::Core;
using namespace Motion::Core::IO;
using namespace Motion::Core::Robot;

namespace Configuration
{
// Replace these values for the target robot. Keep all distance units consistent.
constexpr uint16_t EncoderResolution = 350;
constexpr float RightWheelRadius = 25.0f;
constexpr float LeftWheelRadius = 25.0f;
constexpr float WheelSpacing = 120.0f;

// Motion profile limits in the same distance and time units as the wheels.
constexpr float MaximumAcceleration = 300.0f;
constexpr float MaximumVelocity = 500.0f;
constexpr float MinimumVelocity = 30.0f;

// Replace the placeholders through build flags or a private configuration file.
#ifndef MOTION_WIFI_SSID
#define MOTION_WIFI_SSID "replace-with-wifi-ssid"
#endif

#ifndef MOTION_WIFI_PASSWORD
#define MOTION_WIFI_PASSWORD "replace-with-wifi-password"
#endif

constexpr const char* WifiSsid = MOTION_WIFI_SSID;
constexpr const char* WifiPassword = MOTION_WIFI_PASSWORD;
} // namespace Configuration

// Logging transport.
BaseChannelHandle loggerHandle = ESP32Serial::Create(Serial, 921600);

// Differential-drive motors.
GenericDCMotorHandle rightMotorHandle =
    ESP32DCMotor::Create("right motor", {33, 32});
GenericDCMotorHandle leftMotorHandle =
    ESP32DCMotor::Create("left motor", {25, 26});

// Quadrature encoders. Use different GPIO pairs for the two physical encoders.
GenericEncoderHandle rightEncoderHandle =
    ESP32Encoder::Create("right encoder", {17, 16});
GenericEncoderHandle leftEncoderHandle =
    ESP32Encoder::Create("left encoder", {27, 14});

// Wheel and odometry model.
WheelHandle rightWheelHandle = Wheel::Create(
    rightEncoderHandle,
    Configuration::EncoderResolution,
    Configuration::RightWheelRadius);
WheelHandle leftWheelHandle = Wheel::Create(
    leftEncoderHandle,
    Configuration::EncoderResolution,
    Configuration::LeftWheelRadius);
GenericDifferentialDriveOdometryHandle odometryHandle =
    DifferentialDriveOdometry::Create(
        Configuration::WheelSpacing,
        rightWheelHandle,
        leftWheelHandle);

// One controller per wheel.
BaseControllerHandle rightControllerHandle =
    PIDController::Create({0.004f, 0.0f, 0.0003f}, 1.0f, -1.0f);
BaseControllerHandle leftControllerHandle =
    PIDController::Create({0.004f, 0.0f, 0.0003f}, 1.0f, -1.0f);

// Motion profile and composed stop condition.
BaseProfileGeneratorHandle profileGeneratorHandle =
    TrapezoidalProfileGenerator::Create(
        Configuration::MaximumAcceleration,
        Configuration::MaximumVelocity,
        Configuration::MinimumVelocity);
BaseStopConditionHandle settleCondition =
    SettleStopCondition::Create(5.0f, 1);
BaseStopConditionHandle oscillationCondition =
    OscillationStopCondition::Create(3);
BaseStopConditionHandle stopCondition = settleCondition || oscillationCondition;

// Complete differential-drive navigation service.
DifferentialDriveNavigationHandle robotNavigationHandle =
    DifferentialDriveNavigation::Create(
        odometryHandle,
        profileGeneratorHandle,
        stopCondition,
        {rightMotorHandle,
         leftMotorHandle,
         rightControllerHandle,
         leftControllerHandle});

void StartHardware()
{
    rightMotorHandle->Start();
    leftMotorHandle->Start();
    rightWheelHandle->Start();
    leftWheelHandle->Start();
    odometryHandle->Start();
}

bool ConnectWiFi()
{
    WiFi.begin(Configuration::WifiSsid, Configuration::WifiPassword);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }

    return true;
}

void setup()
{
    LOG_START(loggerHandle, LogLevel::TRACE);
    StartHardware();

    if (!ConnectWiFi())
    {
        LOG_ERROR("Wi-Fi connection failed");
        return;
    }

    if (!Motion::Link::StartTCP<ESP32TCP>())
    {
        LOG_ERROR("Motion Link startup failed");
        return;
    }

    // Expose GetDrive, GetPosition, MoveTo, Orient, Move, Turn, and Stop.
    Motion::Link::AttachCallbacks<DifferentialDriveNavigation>(
        robotNavigationHandle);

    // Spin is the blocking Motion Link receive task.
    Motion::Link::Spin();
}

void loop()
{
    // Motion::Link::Spin() owns the communication task and normally does not
    // return. Put telemetry in a separate FreeRTOS task if it is required.
}

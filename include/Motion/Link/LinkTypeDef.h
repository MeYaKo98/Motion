#pragma once
#include <stdint.h>

#pragma pack(push, 1) // Save current alignment and set alignment to 1 byte

namespace Motion::Bridge {

static constexpr const char* Version = "1.0.0";

enum class RobotDrive : uint8_t{
    DifferentialDrive,
    Omnidrive,
    MecanumDrive,
    Unknown = 0xFF
};

enum class Command : uint16_t{
    //LinkSpecificCommand (A total of 32 Link Specific Callback)
    GetStatus,
    BridgeVersion,
    QueueStatus,
    ClearQueue,
    
    //Robot Specifc
    GetDrive = 0x20,
    GetPosition,
    MoveTo,
    Orient,
    Move,
    Turn,
    Stop,

    //UserSpecific (A total of 64 available callaback)
    UserCommandStart = 0xFFC0
};

enum class Status : uint16_t{
    OK = 0,
    UnkownCommand,
    InvalidRequestLength,
    QueueFull,
    UnknownError = 0xFFFF
};

struct Request {
    uint16_t command;
    uint16_t length;
    uint8_t data[];
};

struct Response {
    Motion::Bridge::Status status;
    uint16_t length;
    uint8_t data[];
};

struct Position {
    float x;
    float y;
    float theta;
};

struct Point {
    float x;
    float y;
};

#pragma pack(pop) // Restore original alignment

}
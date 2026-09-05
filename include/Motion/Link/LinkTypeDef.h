/**
 * @file LinkTypeDef.h
 * @brief Wire-level types for the Motion Link bridge protocol.
 * @details
 * Motion Link transports one packed request followed by one packed response over
 * a Motion Framework communication channel. The protocol is intentionally small:
 * the command identifies the operation, `length` identifies the payload size,
 * and `data` contains the command-specific bytes.
 *
 * Values are copied directly to and from the channel. Consequently, both peers
 * must agree on integer size, floating-point representation, byte order, and the
 * payload format for each command. This header uses one-byte structure packing
 * so the fixed fields do not contain compiler-inserted padding.
 *
 * @warning The current protocol does not define byte-order conversion, framing,
 *          checksums, or version negotiation beyond the `BridgeVersion` command.
 *          Use a compatible Motion Link peer and preserve the packed layouts.
 *
 * **Protocol Frame Layout:**
 * - Request: `[command: uint16_t][length: uint16_t][data: length bytes]`
 * - Response: `[status: uint16_t][length: uint16_t][data: length bytes]`
 *
 * **Payload Conventions:**
 * - `GetPosition` returns one `Position` structure
 * - `MoveTo` accepts one `Point` structure
 * - `Orient`, `Move`, and `Turn` accept one `float`
 * - `GetDrive` returns one `RobotDrive` value
 * - Status-only commands return `length == 0`
 *
 * @note The flexible array members are views over caller-provided buffers. The
 *       structures do not allocate, own, or validate their payload storage.
 */

#pragma once
#include <stdint.h>

#pragma pack(push, 1) // Save current alignment and set alignment to 1 byte

namespace Motion::Bridge {

/**
 * @brief Protocol version reported by the built-in `BridgeVersion` command.
 * @details The response contains `strlen(Version)` bytes and does not include
 *          a terminating null byte.
 */
static constexpr const char* Version = "1.0.0";

/**
 * @brief Drive configurations understood by Motion Link clients.
 * @details The value is serialized as one byte because the underlying type is
 *          `uint8_t`.
 */
enum class RobotDrive : uint8_t{
    /** @brief Two independently driven wheels or wheel groups. */
    DifferentialDrive,
    /** @brief Drive system capable of planar holonomic motion. */
    Omnidrive,
    /** @brief Four-wheel mecanum drive configuration. */
    MecanumDrive,
    /** @brief Drive type is not known or is not supported. */
    Unknown = 0xFF
};

/**
 * @brief Identifies a Motion Link operation.
 * @details
 * Commands below `UserCommandStart` are reserved for the framework. Values
 * from `UserCommandStart` through `0xFFFF` are available for application
 * callbacks. A command is registered as either a fast or slow callback on the
 * device and should use the same numeric value on the client.
 */
enum class Command : uint16_t{
    // Link-specific commands.
    /** @brief Tests that the bridge is available. */
    GetStatus,
    /** @brief Returns the bridge version string bytes. */
    BridgeVersion,
    /** @brief Reports whether the slow-command queue is empty. */
    QueueStatus,
    /** @brief Removes all requests waiting in the slow-command queue. */
    ClearQueue,
    
    // Robot-specific commands.
    /** @brief Returns the connected robot's drive type. */
    GetDrive = 0x20,
    /** @brief Returns the current robot position. */
    GetPosition,
    /** @brief Requests movement to a Cartesian point. */
    MoveTo,
    /** @brief Requests an absolute orientation. */
    Orient,
    /** @brief Requests movement by a linear distance. */
    Move,
    /** @brief Requests rotation by an angular amount. */
    Turn,
    /** @brief Requests the external stop condition to be asserted. */
    Stop,

    // Application-specific commands begin at 0xFFC0.
    /** @brief First command value reserved for application-defined callbacks. */
    UserCommandStart = 0xFFC0
};

/**
 * @brief Result returned in every Motion Link response.
 * @details A response always contains a status and may optionally contain a
 *          payload. Unknown or malformed requests are acknowledged with a
 *          non-OK status and no payload.
 */
enum class Status : uint16_t{
    /** @brief Request completed or was accepted successfully. */
    OK = 0,
    /** @brief No callback is registered for the requested command. */
    UnkownCommand,
    /** @brief Declared request payload exceeds the received frame. */
    InvalidRequestLength,
    /** @brief The slow-command queue cannot accept another request. */
    QueueFull,
    /** @brief An unspecified bridge or channel error occurred. */
    UnknownError = 0xFFFF
};

/**
 * @brief Header and payload of a Motion Link request.
 * @details
 * The wire representation is `command`, `length`, then exactly `length` bytes
 * of payload. The flexible array member is a view into the receive buffer and
 * does not allocate storage. A request containing no payload has `length == 0`.
 *
 * @note Validate `length` against the command's expected payload size before
 *       reading `data`.
 * @note `command` is transmitted as a numeric `uint16_t`; cast to
 *       `Motion::Bridge::Command` after validating or dispatching it.
 */
struct Request {
    uint16_t command;
    uint16_t length;
    uint8_t data[];
};

/**
 * @brief Header and payload of a Motion Link response.
 * @details
 * The wire representation is `status`, `length`, then exactly `length` bytes
 * of response payload. Responses are sent immediately after dispatching a
 * request; slow callbacks receive their acknowledgement before their work is
 * completed.
 * @note Set `length` to the number of payload bytes, not the total response
 *       frame size.
 */
struct Response {
    Motion::Bridge::Status status;
    uint16_t length;
    uint8_t data[];
};

/**
 * @brief Position payload used by the `GetPosition` response.
 * @details `x` and `y` are linear coordinates and `theta` is the robot heading.
 *          Units follow the odometry implementation, typically meters and
 *          radians.
 */
struct Position {
    float x;
    float y;
    float theta;
};

/**
 * @brief Cartesian point payload used by the `MoveTo` request.
 * @details The point uses the same coordinate units as the robot odometry.
 */
struct Point {
    float x;
    float y;
};

#pragma pack(pop) // Restore original alignment

}
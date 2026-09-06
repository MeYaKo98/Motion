# Motion Link Protocol

## Scope

Motion Link is a request/response protocol carried by a Motion Framework communication channel. The bridge defines the message contents; TCP, serial, and channel-specific code define how bytes are delivered.

The protocol version is currently `1.0.0`.

Link allocates 64-byte request and response buffers. With the four-byte fixed header, the practical maximum payload is 60 bytes. Custom payloads must fit within that limit.

## Frame layout

All bridge structures are packed to one-byte alignment.

### Request

| Field | Type | Meaning |
|---|---|---|
| `command` | `uint16_t` | Numeric command identifier |
| `length` | `uint16_t` | Number of payload bytes |
| `data` | `uint8_t[]` | Command payload |

### Response

| Field | Type | Meaning |
|---|---|---|
| `status` | `Motion::Bridge::Status` | Result or queue acknowledgement |
| `length` | `uint16_t` | Number of response payload bytes |
| `data` | `uint8_t[]` | Response payload |

The flexible array members point into the channel buffer. They do not allocate memory. A frame with no payload has `length == 0`.

## Transport assumptions

The current Link implementation treats one successful `Read()` result as one complete request. The channel must preserve message boundaries or buffer and assemble frames before returning them to Link. Raw TCP and serial reads may split or combine frames, so a production adapter must add framing or reassembly. Link does not add a start marker, end marker, checksum, encryption layer, or byte-order conversion.

Both peers must agree on:

- integer sizes and byte order;
- IEEE-754 floating-point representation for `float` payloads;
- the units used by position and distance values;
- the exact payload size for each command; and
- the command and response status numeric values.

## Commands

| Command | Direction | Payload | Response | Callback path |
|---|---|---|---|---|
| `GetStatus` | Request | None | `Status::OK`, no payload | Fast |
| `BridgeVersion` | Request | None | ASCII version bytes, no null terminator | Fast |
| `QueueStatus` | Request | None | `Status::OK` when empty, otherwise `Status::QueueFull` | Fast |
| `ClearQueue` | Request | None | Status only | Fast |
| `GetDrive` | Request | None | `RobotDrive` | Fast |
| `GetPosition` | Request | None | `Position` | Fast |
| `MoveTo` | Request | `Point` | Queue acknowledgement | Slow |
| `Orient` | Request | `float` | Queue acknowledgement | Slow |
| `Move` | Request | `float` | Queue acknowledgement | Slow |
| `Turn` | Request | `float` | Queue acknowledgement | Slow |
| `Stop` | Request | None | Status only | Fast |
| `0xFFC0` to `0xFFFF` | Application | Application-defined | Fast or queued | Application-defined |

`QueueStatus` reports queue availability using the existing `Status` values. It is a coarse status check, not a count of remaining slots.

The built-in service adapters validate that required payload data is present, but the current implementation may accept oversized payloads and ignore trailing bytes. Clients should send the documented size exactly.

## Payload types

### `RobotDrive`

The drive type is serialized as one byte:

- `DifferentialDrive`
- `Omnidrive`
- `MecanumDrive`
- `Unknown` (`0xFF`)

### `Position`

```cpp
struct Position
{
    float x;
    float y;
    float theta;
};
```

`x` and `y` use the units configured by the odometry implementation, typically meters. `theta` is the heading, typically radians.

### `Point`

```cpp
struct Point
{
    float x;
    float y;
};
```

`Point` uses the same coordinate units as the robot odometry.

## Status values

| Status | Meaning |
|---|---|
| `OK` | Request completed or was accepted into the slow queue |
| `UnkownCommand` | No callback is registered for the command |
| `InvalidRequestLength` | Declared payload extends beyond the received frame |
| `QueueFull` | The slow-command queue cannot accept the request |
| `UnknownError` | Unspecified bridge or channel failure |

The spelling `UnkownCommand` is part of the existing public enum and is preserved for API compatibility.

## Custom command example

```cpp
constexpr uint16_t CalibrateCommand =
    static_cast<uint16_t>(Motion::Bridge::Command::UserCommandStart);

Motion::Link::AttachFastCallback(
    CalibrateCommand,
    [](Motion::Bridge::Request* request, Motion::Bridge::Response* response)
    {
        if (request->length != 0)
        {
            response->status = Motion::Bridge::Status::InvalidRequestLength;
            response->length = 0;
            return;
        }

        // Perform only short, non-blocking work here.
        response->status = Motion::Bridge::Status::OK;
        response->length = 0;
    });
```

For an operation that may block or take time, use `AttachSlowCallback`. The client should treat `Status::OK` as an acceptance acknowledgement and use a separate application-defined status or event command if it needs completion reporting. The built-in `Stop` mapping is provided by `ExternalStopCondition`; attaching navigation alone does not expose `Stop`.

## Client implementation checklist

1. Serialize the request header using the exact field order.
2. Set `length` to payload bytes only.
3. Send one complete request frame.
4. Read one complete response frame.
5. Validate the response length before decoding `data`.
6. Treat slow-command `OK` as queued, not completed.
7. Reject unsupported versions or payload layouts before issuing motion commands.

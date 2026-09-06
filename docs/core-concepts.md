# Core Concepts

## Handles and ownership

Factories return framework handle types, normally `std::shared_ptr` aliases. Handles make dependencies explicit and allow services to share hardware objects without transferring ownership through raw pointers.

Keep dependency handles alive for the entire lifetime of the object that uses them:

- Wheels depend on encoder handles.
- Odometry depends on wheel handles.
- Navigation depends on odometry, profile, stop condition, motors, and controllers.
- Link callbacks may capture service handles and require those handles to remain valid while Link runs.

Factories validate many inputs and may throw for invalid configurations. Treat startup as fallible and report failures before enabling motion.

## Start and stop lifecycle

Construction creates configuration and synchronization state; it does not generally start hardware or background work. The common lifecycle is:

1. Create a handle with the component factory.
2. Call `Start()` on hardware and periodic services.
3. Use the service through its public interface.
4. Call `Stop()` before shutdown or allow the documented destructor cleanup.

The application must start motors, encoders, wheels, and odometry explicitly. Navigation is composed from these services and does not provide a separate `Start()` method.

## FreeRTOS and concurrency

Several components own FreeRTOS tasks or synchronization objects:

- `BaseOdometry` periodically calls the drive-specific update method.
- `Motion::Link` creates a worker task for slow callbacks.
- Position and robot state access is protected according to the owning class contract.

Keep high-frequency callbacks short. Avoid blocking, allocation, logging bursts, and long peripheral operations in fast control paths. Do not call blocking framework methods from an interrupt service routine.

## Units and coordinate frames

The framework preserves the units selected by the application. Wheel radius, wheel spacing, odometry coordinates, motion profile limits, and navigation distances must use one consistent linear unit. Angular values should use one consistent angular unit, normally radians.

The library does not convert millimeters to meters automatically. Document the chosen unit system in the application configuration and use named constants instead of unexplained literals.

## Errors and validation

Use factory validation and lifecycle return values as startup gates:

- Check `Start()` results for hardware and channels.
- Validate pins and configuration values before deploying.
- Confirm handles are valid before composing dependent services.
- Treat a Link slow-command `Status::OK` as queue acceptance, not motion completion.
- Keep an external safety path independent from remote commands.

## Extension style

Prefer adding a new implementation behind an existing interface. For a new robot service, expose a narrow base interface, provide a validated factory, define its lifecycle and threading behavior, and add a Link callback specialization only when remote access is part of the contract.

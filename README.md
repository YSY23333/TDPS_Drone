This is the C source code for Team Design Project Skills (TDPS).
In this project, it is required to build a drone which can autonomously take off, fly and land.
# Auto Flight Version

This code is changed from the original quadcopter flight control project.

The main change is in `code/HAL/control.c`. Instead of using a real remote controller, the code changes the remote control variables by itself, such as:

- `Remote.thr`
- `Remote.pitch`
- `Remote.roll`
- `Remote.yaw`

These variables are used like normal remote controller input. Then the old PID control, height control, flow control, and motor control still work as before.

## Main Idea

The auto flight code is in:

```text
code/HAL/control.c
```

The main function is:

```c
AutoFlightManager();
```

This function is called in `code/USER/scheduler.c` every 3ms.

In `code/USER/main.c`, `RC_Analy()` is commented, so the remote controller data will not overwrite the auto values.

## Auto Flight Steps

Current auto flight state machine:

1. `AUTO_WAIT` - wait after power on
2. `AUTO_TAKEOFF` - unlock and take off
3. `AUTO_HOVER` - hover with height lock and flow control
4. `AUTO_FORWARD` - fly forward
5. `AUTO_LEFTWARD` - fly left
6. `AUTO_LAND` - land
7. `AUTO_FINISH` - lock motors and finish

The time is controlled by `auto_counter`.

Because `AutoFlightManager()` runs every 3ms:

```text
time = auto_counter * 3ms
```

For example, `auto_counter > 1000` is about 3 seconds.

## How To Change Flight Action

Most changes are in `code/HAL/control.c`.

Useful variables:

- `Remote.thr` - throttle
- `Remote.pitch` - forward/backward
- `Remote.roll` - left/right
- `Remote.yaw` - yaw
- `Angle.yaw` - yaw angle
- `ALL_flag.unlock` - unlock flag
- `ALL_flag.height_lock` - height lock flag
- `ALL_flag.flow_control` - optical flow control flag
- `auto_counter` - step time
- `auto_flight_state` - current auto flight state

Remote values are usually from 1000 to 2000. Middle value is about 1500.

Change values slowly when testing.

## Important Files

```text
code/HAL/control.c      auto flight and control logic
code/HAL/control.h      control function declarations
code/HAL/remote.c       old remote controller code
code/USER/main.c        main loop
code/USER/scheduler.c   3ms task scheduler
code/USER/ALL_DATA.h    Remote, Angle, ALL_flag data
```

## Build

Open this Keil project:

```text
code/mini.uvprojx
```

Build it in Keil uVision, then flash the generated hex file to the flight controller board.

## Safety

- Test without propellers first.
- Make sure motors do not start suddenly.
- Use small throttle changes first.
- Prepare a way to cut power.
- Check height sensor, optical flow, and attitude data before real flight.


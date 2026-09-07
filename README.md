# MRS UAV ArduPilot API Plugin

[MRS HW Api](https://github.com/ctu-mrs/mrs_uav_hw_api) plugin for interfacing the [UAV Core](https://github.com/ctu-mrs/mrs_uav_core) with the **ArduPilot** autopilot (developed and tested against **ArduPilot firmware 4.6.3**).

It is a sibling of [`mrs_uav_px4_api`](https://github.com/ctu-mrs/mrs_uav_px4_api): it implements the same `mrs_uav_hw_api::MrsUavHwApi` plugin interface, reuses MAVROS as the MAVLink transport, and launches the hardware API through an analogous `launch/api.launch.py`.

## How it works

`launch/api.launch.py` composes the `mrs_uav_hw_api::HwApiManager` node (from `mrs_uav_hw_api`) and loads this package's `MrsUavArdupilotApi` plugin into it via `pluginlib`. The manager remaps the plugin's `~/mavros_*` topics onto the standard MAVROS topics. When not in simulation, the real-world launch additionally starts a `mavros_node` configured for an ArduPilot FCU.

## Required ArduPilot configuration

This plugin sends attitude / attitude-rate setpoints to MAVROS' `setpoint_raw/attitude` topic, which MAVROS forwards to the FCU as `SET_ATTITUDE_TARGET`.

> :warning: **By default ArduPilot interprets `SET_ATTITUDE_TARGET.thrust` in GUIDED mode as a normalized climb-rate command ([0..1], 0.5 ≈ hover), NOT a direct thrust value.**
>
> To get **direct (pass-through) thrust control** — matching the semantics the MRS UAV System expects from this API — set the following ArduPilot parameter:
>
> ```
> GUID_OPTIONS = 8   # bit 3 (value 8) -> "Pass throttle as pass-through thrust" in GUIDED
> ```
>
> See the [ArduPilot Copter Commands in Guided Mode](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html) documentation for details. Without `GUID_OPTIONS=8`, the throttle reference coming from the MRS controllers will be interpreted as a climb-rate command and the UAV will not behave as intended.

Other recommended ArduPilot params for offboard-style control:

```
ARMING_CHECK     = 1       # keep pre-arm checks enabled
DISARM_DELAY     = 0       # optional: don't auto-disarm on landing
FS_THR_ENABLE    = 0       # avoid radio-throttle failsafe fighting offboard
```

## Supported inputs / outputs

See `config/ardupilot_api.yaml` for the capability flags. The defaults are:

**Control inputs** (consumed by `callback*Cmd`):
- `attitude_rate` — enabled (body rates + thrust → `SET_ATTITUDE_TARGET`)
- `attitude` — enabled (orientation + thrust → `SET_ATTITUDE_TARGET`)
- `control_group` — **disabled by default**. ArduPilot has no PX4-equivalent `actuator_control` / `SET_ACTUATOR_CONTROL_TARGET` path; the corresponding callbacks log and return `false`. Enable only if you implement a custom mapping (e.g. RC override).
- Direct actuator command (`callbackActuatorCmd`) — not supported, returns `false`.

**Outputs** (published from MAVROS topics): distance sensor, GNSS, GNSS status, IMU, altitude, magnetometer heading, magnetic field, RC channels, RC RSSI, battery state, position, orientation, velocity, angular velocity, odometry, ground truth (simulation). RTK is left disabled (TODO).

## Offboard / arming

- `callbackOffboard` switches the FCU to ArduPilot's **`GUIDED`** mode (via MAVROS `set_mode`). This is the ArduPilot equivalent of PX4's `OFFBOARD` mode.
- `callbackArming` uses `MAV_CMD_COMPONENT_ARM_DISARM` (command 400), which works the same on ArduPilot as on PX4. As in the PX4 plugin, arming by service is only permitted in simulation — in the real world the UAV must be armed by the RC controller.

## Local position / frame convention

Unlike PX4 (which applies an azimuthal-equidistant projection to WGS84 for its local EKF origin), ArduPilot's `local_position` is reported directly in the EKF origin frame. This plugin therefore publishes the MAVROS `local_position/odom` (and `global_position/global`) data more directly, without the PX4-specific reverse-projection step. The GNSS→UTM reference initialization is retained for consistency with the rest of the MRS UAV System. Fine-tuning the exact frame convention to your setup is left to the user.

## Environment variables

| Variable | Default | Description |
|---|---|---|
| `UAV_NAME` | `uav1` | ROS namespace of the UAV |
| `RUN_TYPE` | `simulation` | `simulation` or `realworld` (sets the `simulation` launch arg) |
| `USE_SIM_TIME` | `false` | Use ROS sim time |
| `ARDUPILOT_IP` | (empty) | If set, MAVROS connects over UDP: `udp://:14550@<IP>:14550`. Otherwise serial. |
| `ARDUPILOT_SERIAL` | `/dev/ttyACM0` | Serial device for the FCU (when `ARDUPILOT_IP` is empty) |
| `ARDUPILOT_BAUD` | `115200` | Serial baudrate for the FCU |

## Usage

Simulation (composes the hw_api; expects an external/simulated MAVROS or FCU):

```bash
RUN_TYPE=simulation ros2 launch mrs_uav_ardupilot_api api.launch.py
```

Real world (composes the hw_api AND starts MAVROS against the FCU):

```bash
RUN_TYPE=realworld ros2 launch mrs_uav_ardupilot_api api.launch.py
# or override the connection:
RUN_TYPE=realworld ARDUPILOT_IP=192.168.1.2 ros2 launch mrs_uav_ardupilot_api api.launch.py
```

A custom config can be supplied via the `custom_config` launch argument (absolute path, or relative to the current working directory).

# F407 Four-Axis Robotic Arm Lower Controller

## Project entry

- Keil project: `MDK-ARM/F407-RoboticArm.uvprojx`
- CubeMX configuration: `F407-RoboticArm.ioc`
- Build log: `MDK-ARM/build_roboticarm.log`
- Firmware output: `MDK-ARM/F407-RoboticArm/F407-RoboticArm.hex`

## Axis and bus mapping

| Axis | ZDT address | Mechanism |
| --- | ---: | --- |
| 1 | 1 | Bottom lead screw |
| 2 | 2 | Base rotation |
| 3 | 3 | Upper arm |
| 4 | 4 | Forearm |

USART1 uses PA9/PA10 at 115200 8N1. Confirm that all four drivers are the
correct TTL multi-drop version before connecting them to one bus. Do not wire
several ordinary push-pull TX outputs together. Use an RS485 physical layer or
separate buses when the actual driver interface cannot support a reliable
multi-node TTL topology.

## Safe startup

1. Power on with the mechanism unloaded or mechanically supported.
2. The firmware queues disable commands for all four axes and performs no
   automatic motion.
3. Verify all four motors show online on the OLED.
4. Move the arm by hand to the chosen safe reference pose.
5. Open the zero page and confirm each axis with K6, or confirm all with K3.
6. Press K4 only after all four zero flags are valid.
7. Test one axis at low speed before assembling or loading the complete arm.

## Controls

- K1/K2: move between pages; select an axis on the zero page; select the global
  encoder mode on the mode page.
- K3 short: confirm the current menu item; zero all axes on the zero page.
- K3 long: return to the home page.
- K4 short: enable all axes after all four zero confirmations.
- K4 long: disable all axes.
- K5 short: stop and lock the selected axis.
- K5 long: stop and lock all axes.
- K6 short: stop the selected axis and reset its current driver position to zero.

The four encoders always address axes 1-4 regardless of the current OLED page.
The global encoder mode selects angle, position-command speed, or acceleration.
Speed and acceleration changes are RAM-only and take effect on the next angle
command.

## Hardware validation still required

- Confirm motor addresses 1-4 and 115200 baud one motor at a time.
- Confirm positive direction, transmission ratio, and mechanical travel for
  every joint at low speed and without payload.
- Replace the initial +/-180 degree software range with measured safe limits.
- Add physical limit switches and a hardware emergency stop before automatic
  trajectories, ROS, or loaded operation.
- Gravity-loaded joints may fall when disabled; use mechanical support, a brake,
  self-locking transmission, or counterbalance as required.

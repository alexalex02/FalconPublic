# Tunable Auto Align

How the on-demand, tunable auto align sequence works and how to adjust it live.

## Flow
- The trigger `/Tuning/AutoAlign/DriveToTunablePose` is watched in `src/main/java/frc/robot/RobotContainer.java`. When set true (Elastic/AdvantageScope), it schedules `AutoAlignCommands.driveToTunablePoseProfiled` and clears the trigger after the command finishes.
- The command (see `src/main/java/frc/robot/commands/AutoAlignCommands.java`) reads the target pose every loop from the entries in `AutoAlignTunables`.
- XY translation uses a `ProfiledPIDController` on the scalar distance to the target; the output is pointed along the current-to-target vector. Velocity/accel constraints come from the tunables and are clamped to the drivetrain's max linear speed.
- Heading uses a separate continuous-input profiled PID with tunable velocity/accel limits, also clamped to the drivetrain's angular max.
- If the target pose changes mid-run, the translation controller is re-seeded to avoid a jump in motion while it retimes to the new goal.
- The command ends once both profiled controllers report `atGoal()`. AdvantageKit logs `AutoAlign/Active`, `AutoAlign/TargetPose`, and `AutoAlign/Tunables/*` for visibility.

## Tunables (NetworkTables paths and defaults)
- Target pose (field coordinates): `/Tuning/AutoAlign/TargetPoseX` (m, 0.0), `/Tuning/AutoAlign/TargetPoseY` (m, 0.0), `/Tuning/AutoAlign/TargetPoseThetaDeg` (deg, 0.0).
- Translation gains/limits: `/Tuning/AutoAlign/XYkP` (5.0), `/Tuning/AutoAlign/XYMaxVelMps` (3.0), `/Tuning/AutoAlign/XYMaxAccMps2` (3.0).
- Heading gains/limits: `/Tuning/AutoAlign/ThetaKp` (5.0), `/Tuning/AutoAlign/ThetaMaxVelRadPerSec` (8.0), `/Tuning/AutoAlign/ThetaMaxAccRadPerSec2` (20.0).
- Run trigger: `/Tuning/AutoAlign/DriveToTunablePose` (boolean, default false). Set true to start a run; it self-resets afterward.

## How to use and tune
1. In Elastic/AdvantageScope, open `/Tuning/AutoAlign`.
2. Set the target pose in field coordinates (meters from the WPILib field origin; theta is CCW degrees from +X).
3. Start with conservative limits (e.g., about 1.5 m/s, 1.5 m/s^2, 4 rad/s, 8 rad/s^2) and modest kP (3-4) to prevent oscillation.
4. Flip `DriveToTunablePose` to true. The command will take control of the drive subsystem until both controllers are at goal.
5. Watch `AutoAlign/Active`, `AutoAlign/TargetPose`, and the swerve setpoint logs in AdvantageScope. Raise kP and motion limits gradually until the response is crisp without overshoot.
6. Adjusting any tunable or the target pose mid-run updates the controllers immediately; re-trigger as needed to try new settings.

## Using graphs to tune
- Overlay target vs actual pose: plot `AutoAlign/TargetPose` and `Odometry/Robot` (x, y, heading) to see overshoot or steady-state error. Ideally position and heading converge cleanly with no oscillation.
- Watch controller activity: graph `AutoAlign/Active` as a digital trace to see when the command runs; use it as a marker for start/stop when reviewing motion.
- Check velocity profiles: view `SwerveChassisSpeeds/Setpoints` vs `SwerveChassisSpeeds/Measured` to confirm the max vel/acc limits produce smooth ramps without clipping.
- Spot oscillations: if x/y or heading ring around the target, lower the corresponding kP or the max velocity/accel limits. If it lags or never reaches the target, increase kP or allow more velocity/accel.
- Use short moves: tune with small step changes in target pose so graphs stay easy to read, then validate on longer moves once the shape looks clean.

## About derivative (D)
- The auto-align profiled PIDs currently run with only kP (kD = 0). The trapezoidal motion constraints already shape the accel/jerk, so D is often unnecessary.
- If you still see oscillation after lowering kP or tightening motion limits, adding a small kD term can dampen ringing (especially on heading). Graph heading/position error and controller output to verify D reduces overshoot without adding noise.
- Avoid large kD values: derivative amplifies sensor noise and can fight the motion profile. Start near zero and bump in small steps while watching the graphs.

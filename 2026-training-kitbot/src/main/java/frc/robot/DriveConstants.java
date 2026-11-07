package frc.robot;

public final class DriveConstants {
  private DriveConstants() {}

  public static final int LEFT_LEADER_ID = 1;
  public static final int LEFT_FOLLOWER_ID = 2;
  public static final int RIGHT_LEADER_ID = 3;
  public static final int RIGHT_FOLLOWER_ID = 4;
  public static final int PIGEON_ID = 5;

  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean RIGHT_SIDE_INVERTED = true;
  public static final boolean FOLLOWER_OPPOSES_LEADER = false;
  public static final double VOLTAGE_COMPENSATION_VOLTS = 12.0;
  public static final double TELEOP_RAMP_SECONDS = 0.15;
  public static final double TELEOP_MAX_OUTPUT = 0.45;//0.6

  public static final double DEADBAND = 0.08;

  // Teleop heading hold (TeleopDrive): keeps the robot straight while only the left stick is used.
  // Correction added to the turn, per degree off. Raise if it still drifts, lower if it wobbles.
  public static final double TELEOP_HEADING_KP = 0.03;
  // Most turn the correction may ever add (so it can never out-steer the driver much).
  public static final double TELEOP_MAX_CORRECTION = 0.25;
  // After letting go of the turn stick, wait this long for the robot to stop rotating before
  // locking the heading in (otherwise it snaps back to where it was mid-turn).
  public static final double HEADING_HOLD_SETTLE_SECONDS = 0.25;

  // true = the whole drivetrain is wired backwards (positive motor output drives the robot
  // backward). Drivetrain flips motor outputs AND encoder readings, so everywhere else in the code
  // positive = forward, positive rotation = turn left (CCW), as WPILib expects.
  public static final boolean DRIVE_REVERSED = true;

  // Each encoder plugs into the data port on top of a SPARK MAX (not the roboRIO DIO).
  // Set these to the CAN ID of the SPARK MAX that each encoder cable goes into.
  public static final int LEFT_ENCODER_MOTOR_ID = LEFT_LEADER_ID;
  public static final int RIGHT_ENCODER_MOTOR_ID = RIGHT_FOLLOWER_ID;
  public static final boolean LEFT_ENCODER_REVERSED = false;
  public static final boolean RIGHT_ENCODER_REVERSED = true;
  public static final boolean GYRO_REVERSED = false;

  public static final double ENCODER_CYCLES_PER_REV = 2048;
  public static final double ENCODER_TURNS_PER_WHEEL_TURN = 1.0;
  public static final double WHEEL_DIAMETER_INCHES = 5.75;
  public static final double CALIBRATION_FACTOR = 11;

  // The SPARK MAX counts every edge of both A and B channels: 4 counts per cycle.
  public static final int ENCODER_COUNTS_PER_REV = (int) (ENCODER_CYCLES_PER_REV * 4);

  // The SPARK MAX reports encoder rotations; this turns one rotation into inches driven.
  public static final double INCHES_PER_ENCODER_REV =
      (Math.PI * WHEEL_DIAMETER_INCHES) / ENCODER_TURNS_PER_WHEEL_TURN * CALIBRATION_FACTOR;

  public static final double DISTANCE_KP = 0.02;
  public static final double DISTANCE_KD = 0.002;

  public static final double MAX_VELOCITY_IN_PER_SEC = 50.0;
  public static final double MAX_ACCEL_IN_PER_SEC2 = 50.0;
  public static final double KS_VOLTS = 0.72;
  public static final double KV_VOLTS_PER_IN_PER_SEC = 0.084;
  public static final double KA_VOLTS_PER_IN_PER_SEC2 = 0.0;
  public static final double HEADING_KP = 0.02;
  public static final double DISTANCE_TOLERANCE_INCHES = 1.0;
  public static final double VELOCITY_TOLERANCE_IN_PER_SEC = 2.0;
  public static final double AUTO_TIMEOUT_SECONDS = 6.0;

  // Simple "drive until the encoders say 10 ft" auton (SimpleDriveStraight).
  public static final double AUTO_DISTANCE_INCHES = 120.0;
  // Slow on purpose: auto is 10 s and slower = less wheel slip and less coasting past the line.
  // 0.25 is about 3 V (~2.5 ft/s); finishes 10 ft in about 6 s.
  public static final double SIMPLE_AUTO_SPEED = 0.25;
  // Must stay well above kS (KS_VOLTS / 12 is about 0.06) or the robot stalls before the line.
  public static final double SIMPLE_AUTO_SLOW_SPEED = 0.13;
  public static final double SIMPLE_AUTO_SLOW_ZONE_INCHES = 24.0;
  // Kept under the 10 s auto period so the robot always stops on its own.
  public static final double SIMPLE_AUTO_TIMEOUT_SECONDS = 9.5;
  public static final double ENCODER_STRAIGHT_KP = 0.05;
}

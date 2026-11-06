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
  public static final double SYSID_STEP_VOLTS = 4.0;

  // Simple "drive until the encoders say 10 ft" auton (SimpleDriveStraight).
  public static final double AUTO_DISTANCE_INCHES = 120.0;
  public static final double SIMPLE_AUTO_SPEED = 0.4;
  public static final double SIMPLE_AUTO_SLOW_SPEED = 0.2;
  public static final double SIMPLE_AUTO_SLOW_ZONE_INCHES = 18.0;
  public static final double ENCODER_STRAIGHT_KP = 0.05;
}

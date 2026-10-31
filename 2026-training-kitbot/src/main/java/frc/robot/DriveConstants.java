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
  public static final double TELEOP_MAX_OUTPUT = 0.6;
  public static final double DEADBAND = 0.08;
  public static final boolean INVERT_TURN = false;

  public static final int[] LEFT_ENCODER_PORTS = {0, 1};
  public static final int[] RIGHT_ENCODER_PORTS = {2, 3};
  public static final boolean LEFT_ENCODER_REVERSED = false;
  public static final boolean RIGHT_ENCODER_REVERSED = true;
  public static final boolean GYRO_REVERSED = false;

  public static final double ENCODER_CYCLES_PER_REV = 2048;
  public static final double ENCODER_TURNS_PER_WHEEL_TURN = 1.0;
  public static final double WHEEL_DIAMETER_INCHES = 6.0;
  public static final double CALIBRATION_FACTOR = 1.0;

  public static final double INCHES_PER_CYCLE =
      (Math.PI * WHEEL_DIAMETER_INCHES)
          / (ENCODER_CYCLES_PER_REV * ENCODER_TURNS_PER_WHEEL_TURN)
          * CALIBRATION_FACTOR;

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
}

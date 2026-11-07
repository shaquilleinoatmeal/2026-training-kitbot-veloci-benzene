package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private final SparkMax leftLeader =
      new SparkMax(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushed);
  private final SparkMax leftFollower =
      new SparkMax(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushed);
  private final SparkMax rightLeader =
      new SparkMax(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushed);
  private final SparkMax rightFollower =
      new SparkMax(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushed);

  // -1 when the drivetrain is wired backwards (see DriveConstants.DRIVE_REVERSED). Applied to the
  // motor outputs and the encoder readings, so the rest of the code always uses positive = forward.
  private static final double DRIVE_SIGN = DriveConstants.DRIVE_REVERSED ? -1.0 : 1.0;

  private double leftOutput;
  private double rightOutput;
  private final DifferentialDrive drive =
      new DifferentialDrive(
          output -> {
            leftOutput = output;
            leftLeader.set(DRIVE_SIGN * output);
          },
          output -> {
            rightOutput = output;
            rightLeader.set(DRIVE_SIGN * output);
          });

  private final Pigeon2 pigeon = new Pigeon2(DriveConstants.PIGEON_ID);

  // The encoders are plugged into SPARK MAX data ports, picked by CAN ID in DriveConstants.
  private final SparkMax leftEncoderMotor = motorWithId(DriveConstants.LEFT_ENCODER_MOTOR_ID);
  private final SparkMax rightEncoderMotor = motorWithId(DriveConstants.RIGHT_ENCODER_MOTOR_ID);
  private final RelativeEncoder leftEncoder = leftEncoderMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightEncoderMotor.getEncoder();

  private final DifferentialDriveOdometry odometry;
  private final Field2d field = new Field2d();

  private DifferentialDrivetrainSim driveSim;
  private double simLeftOffsetInches;
  private double simRightOffsetInches;
  private double simHeadingOffsetDegrees;

  public Drivetrain() {
    configure(leftLeader, baseConfig());
    configure(rightLeader, baseConfig().inverted(DriveConstants.RIGHT_SIDE_INVERTED));
    configure(
        leftFollower, baseConfig().follow(leftLeader, DriveConstants.FOLLOWER_OPPOSES_LEADER));
    configure(
        rightFollower, baseConfig().follow(rightLeader, DriveConstants.FOLLOWER_OPPOSES_LEADER));
    drive.setDeadband(0.0);

    configureEncoder(leftEncoderMotor, DriveConstants.LEFT_ENCODER_REVERSED);
    configureEncoder(rightEncoderMotor, DriveConstants.RIGHT_ENCODER_REVERSED);

    odometry =
        new DifferentialDriveOdometry(
            getRotation(),
            Units.inchesToMeters(getLeftInches()),
            Units.inchesToMeters(getRightInches()));
    SmartDashboard.putData("Field", field);

    if (RobotBase.isSimulation()) {
      driveSim =
          DifferentialDrivetrainSim.createKitbotSim(
              KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);
    }
    resetEncoders();
  }

  private SparkMax motorWithId(int canId) {
    for (SparkMax motor : new SparkMax[] {leftLeader, leftFollower, rightLeader, rightFollower}) {
      if (motor.getDeviceId() == canId) {
        return motor;
      }
    }
    throw new IllegalArgumentException(
        "Encoder motor ID " + canId + " is not one of the drive motor IDs in DriveConstants");
  }

  private static void configureEncoder(SparkMax motor, boolean reversed) {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .encoder
        .countsPerRevolution(DriveConstants.ENCODER_COUNTS_PER_REV)
        .inverted(reversed)
        .positionConversionFactor(DriveConstants.INCHES_PER_ENCODER_REV)
        .velocityConversionFactor(DriveConstants.INCHES_PER_ENCODER_REV / 60.0);
    // Applied on top of the motor settings above, so keep them (no reset).
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  private static SparkMaxConfig baseConfig() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(DriveConstants.CURRENT_LIMIT_AMPS);
    config.voltageCompensation(DriveConstants.VOLTAGE_COMPENSATION_VOLTS);
    return config;
  }

  private static void configure(SparkMax motor, SparkBaseConfig config) {
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void arcadeDrive(double forward, double rotation, double maxOutput) {
    arcadeDrive(forward, rotation, maxOutput, true);
  }

  public void arcadeDrive(
      double forward, double rotation, double maxOutput, boolean squareInputs) {
    drive.setMaxOutput(maxOutput);
    drive.arcadeDrive(forward, rotation, squareInputs);
  }

  public void tankDrive(double left, double right) {
    drive.setMaxOutput(1.0);
    drive.tankDrive(left, right, false);
  }

  public void stop() {
    drive.stopMotor();
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    if (driveSim != null) {
      simLeftOffsetInches = Units.metersToInches(driveSim.getLeftPositionMeters());
      simRightOffsetInches = Units.metersToInches(driveSim.getRightPositionMeters());
    }
  }

  // Simulation has no real SPARK MAX encoders, so it reads the simulated drivetrain instead.
  public double getLeftInches() {
    if (driveSim != null) {
      return Units.metersToInches(driveSim.getLeftPositionMeters()) - simLeftOffsetInches;
    }
    return DRIVE_SIGN * leftEncoder.getPosition();
  }

  public double getRightInches() {
    if (driveSim != null) {
      return Units.metersToInches(driveSim.getRightPositionMeters()) - simRightOffsetInches;
    }
    return DRIVE_SIGN * rightEncoder.getPosition();
  }

  public double getLeftInchesPerSecond() {
    if (driveSim != null) {
      return Units.metersToInches(driveSim.getLeftVelocityMetersPerSecond());
    }
    return DRIVE_SIGN * leftEncoder.getVelocity();
  }

  public double getRightInchesPerSecond() {
    if (driveSim != null) {
      return Units.metersToInches(driveSim.getRightVelocityMetersPerSecond());
    }
    return DRIVE_SIGN * rightEncoder.getVelocity();
  }

  public double getAverageInches() {
    return (getLeftInches() + getRightInches()) / 2.0;
  }

  // In simulation the Pigeon's yaw arrives over simulated CAN on a real-time clock, so it lags or
  // goes stale in fast unit tests. Read the simulated drivetrain instead, like the encoders do.
  public double getHeadingDegrees() {
    if (driveSim != null) {
      return driveSim.getHeading().getDegrees() - simHeadingOffsetDegrees;
    }
    double yaw = pigeon.getYaw().getValueAsDouble();
    return DriveConstants.GYRO_REVERSED ? -yaw : yaw;
  }

  private Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public void resetSensors() {
    resetEncoders();
    pigeon.setYaw(0);
    if (driveSim != null) {
      simHeadingOffsetDegrees = driveSim.getHeading().getDegrees();
    }
    odometry.resetPosition(Rotation2d.kZero, 0.0, 0.0, Pose2d.kZero);
  }

  @Override
  public void periodic() {
    odometry.update(
        getRotation(),
        Units.inchesToMeters(getLeftInches()),
        Units.inchesToMeters(getRightInches()));
    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putNumber("Drive/Left inches", getLeftInches());
    SmartDashboard.putNumber("Drive/Right inches", getRightInches());
    SmartDashboard.putNumber("Drive/Average inches", getAverageInches());
    SmartDashboard.putNumber("Drive/Left in per sec", getLeftInchesPerSecond());
    SmartDashboard.putNumber("Drive/Right in per sec", getRightInchesPerSecond());
    SmartDashboard.putNumber("Drive/Heading deg", getHeadingDegrees());
  }

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(withSimFriction(leftOutput), withSimFriction(rightOutput));
    driveSim.update(0.02);

    pigeon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    double simYaw = driveSim.getHeading().getDegrees();
    pigeon.getSimState().setRawYaw(DriveConstants.GYRO_REVERSED ? -simYaw : simYaw);
  }

  private static double withSimFriction(double output) {
    double volts = output * DriveConstants.VOLTAGE_COMPENSATION_VOLTS;
    return Math.signum(volts) * Math.max(0.0, Math.abs(volts) - DriveConstants.KS_VOLTS);
  }
}

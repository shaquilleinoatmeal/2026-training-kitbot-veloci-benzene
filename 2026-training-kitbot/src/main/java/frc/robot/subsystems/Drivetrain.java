package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  private double leftOutput;
  private double rightOutput;
  private final DifferentialDrive drive =
      new DifferentialDrive(
          output -> {
            leftOutput = output;
            leftLeader.set(output);
          },
          output -> {
            rightOutput = output;
            rightLeader.set(output);
          });

  private final Pigeon2 pigeon = new Pigeon2(DriveConstants.PIGEON_ID);

  private final Encoder leftEncoder =
      new Encoder(
          DriveConstants.LEFT_ENCODER_PORTS[0],
          DriveConstants.LEFT_ENCODER_PORTS[1],
          DriveConstants.LEFT_ENCODER_REVERSED);
  private final Encoder rightEncoder =
      new Encoder(
          DriveConstants.RIGHT_ENCODER_PORTS[0],
          DriveConstants.RIGHT_ENCODER_PORTS[1],
          DriveConstants.RIGHT_ENCODER_REVERSED);

  private final DifferentialDriveOdometry odometry;
  private final Field2d field = new Field2d();

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(null, Volts.of(DriveConstants.SYSID_STEP_VOLTS), null),
          new SysIdRoutine.Mechanism(
              volts -> {
                double output = volts.in(Volts) / DriveConstants.VOLTAGE_COMPENSATION_VOLTS;
                tankDrive(output, output);
              },
              log -> {
                log.motor("drive-left")
                    .voltage(Volts.of(leftOutput * DriveConstants.VOLTAGE_COMPENSATION_VOLTS))
                    .linearPosition(Inches.of(getLeftInches()))
                    .linearVelocity(InchesPerSecond.of(getLeftInchesPerSecond()));
                log.motor("drive-right")
                    .voltage(Volts.of(rightOutput * DriveConstants.VOLTAGE_COMPENSATION_VOLTS))
                    .linearPosition(Inches.of(getRightInches()))
                    .linearVelocity(InchesPerSecond.of(getRightInchesPerSecond()));
              },
              this));

  private DifferentialDrivetrainSim driveSim;
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;

  public Drivetrain() {
    configure(leftLeader, baseConfig());
    configure(rightLeader, baseConfig().inverted(DriveConstants.RIGHT_SIDE_INVERTED));
    configure(
        leftFollower, baseConfig().follow(leftLeader, DriveConstants.FOLLOWER_OPPOSES_LEADER));
    configure(
        rightFollower, baseConfig().follow(rightLeader, DriveConstants.FOLLOWER_OPPOSES_LEADER));
    drive.setDeadband(0.0);

    leftEncoder.setDistancePerPulse(DriveConstants.INCHES_PER_CYCLE);
    rightEncoder.setDistancePerPulse(DriveConstants.INCHES_PER_CYCLE);
    resetEncoders();

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
      leftEncoderSim = new EncoderSim(leftEncoder);
      rightEncoderSim = new EncoderSim(rightEncoder);
    }
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
    drive.setMaxOutput(maxOutput);
    drive.arcadeDrive(forward, rotation);
  }

  public void tankDrive(double left, double right) {
    drive.setMaxOutput(1.0);
    drive.tankDrive(left, right, false);
  }

  public void stop() {
    drive.stopMotor();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getLeftInches() {
    return leftEncoder.getDistance();
  }

  public double getRightInches() {
    return rightEncoder.getDistance();
  }

  public double getLeftInchesPerSecond() {
    return leftEncoder.getRate();
  }

  public double getRightInchesPerSecond() {
    return rightEncoder.getRate();
  }

  public double getAverageInches() {
    return (getLeftInches() + getRightInches()) / 2.0;
  }

  public double getHeadingDegrees() {
    double yaw = pigeon.getYaw().getValueAsDouble();
    return DriveConstants.GYRO_REVERSED ? -yaw : yaw;
  }

  private Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public void resetSensors() {
    resetEncoders();
    pigeon.setYaw(0);
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

    leftEncoderSim.setDistance(Units.metersToInches(driveSim.getLeftPositionMeters()));
    rightEncoderSim.setDistance(Units.metersToInches(driveSim.getRightPositionMeters()));
    leftEncoderSim.setRate(Units.metersToInches(driveSim.getLeftVelocityMetersPerSecond()));
    rightEncoderSim.setRate(Units.metersToInches(driveSim.getRightVelocityMetersPerSecond()));
    pigeon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    double simYaw = driveSim.getHeading().getDegrees();
    pigeon.getSimState().setRawYaw(DriveConstants.GYRO_REVERSED ? -simYaw : simYaw);
  }

  private static double withSimFriction(double output) {
    double volts = output * DriveConstants.VOLTAGE_COMPENSATION_VOLTS;
    return Math.signum(volts) * Math.max(0.0, Math.abs(volts) - DriveConstants.KS_VOLTS);
  }
}

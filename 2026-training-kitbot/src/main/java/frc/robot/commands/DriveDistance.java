package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveConstants;
import frc.robot.TunableNumber;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends Command {
  private final Drivetrain drivetrain;
  private final double targetInches;
  private double startInches;
  private double startHeadingDegrees;
  private double lastSetpointVelocity;
  private double headingKp;
  private double toleranceInches;

  private final TunableNumber distanceKp =
      new TunableNumber("Auto distance kP", DriveConstants.DISTANCE_KP);
  private final TunableNumber distanceKd =
      new TunableNumber("Auto distance kD", DriveConstants.DISTANCE_KD);
  private final TunableNumber headingKpTunable =
      new TunableNumber("Auto heading kP", DriveConstants.HEADING_KP);
  private final TunableNumber toleranceTunable =
      new TunableNumber("Auto tolerance inches", DriveConstants.DISTANCE_TOLERANCE_INCHES);
  private final TunableNumber maxVelocity =
      new TunableNumber("Auto max velocity in per sec", DriveConstants.MAX_VELOCITY_IN_PER_SEC);
  private final TunableNumber maxAccel =
      new TunableNumber("Auto max accel in per sec2", DriveConstants.MAX_ACCEL_IN_PER_SEC2);

  private final ProfiledPIDController distancePid =
      new ProfiledPIDController(
          DriveConstants.DISTANCE_KP,
          0.0,
          DriveConstants.DISTANCE_KD,
          new TrapezoidProfile.Constraints(
              DriveConstants.MAX_VELOCITY_IN_PER_SEC, DriveConstants.MAX_ACCEL_IN_PER_SEC2));
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          DriveConstants.KS_VOLTS / DriveConstants.VOLTAGE_COMPENSATION_VOLTS,
          DriveConstants.KV_VOLTS_PER_IN_PER_SEC / DriveConstants.VOLTAGE_COMPENSATION_VOLTS,
          DriveConstants.KA_VOLTS_PER_IN_PER_SEC2 / DriveConstants.VOLTAGE_COMPENSATION_VOLTS);

  public DriveDistance(Drivetrain drivetrain, double targetInches) {
    this.drivetrain = drivetrain;
    this.targetInches = targetInches;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    startInches = drivetrain.getAverageInches();
    startHeadingDegrees = drivetrain.getHeadingDegrees();
    headingKp = headingKpTunable.get();
    toleranceInches = toleranceTunable.get();
    distancePid.setPID(distanceKp.get(), 0.0, distanceKd.get());
    distancePid.setConstraints(new TrapezoidProfile.Constraints(maxVelocity.get(), maxAccel.get()));
    distancePid.setTolerance(toleranceInches, DriveConstants.VELOCITY_TOLERANCE_IN_PER_SEC);
    distancePid.reset(0.0);
    lastSetpointVelocity = 0.0;
  }

  @Override
  public void execute() {
    double traveledInches = drivetrain.getAverageInches() - startInches;
    double pidOutput = distancePid.calculate(traveledInches, targetInches);
    TrapezoidProfile.State setpoint = distancePid.getSetpoint();

    double feedforwardOutput;
    if (setpoint.velocity != 0.0) {
      feedforwardOutput =
          feedforward.calculateWithVelocities(lastSetpointVelocity, setpoint.velocity);
    } else {
      double errorInches = targetInches - traveledInches;
      feedforwardOutput =
          Math.abs(errorInches) > toleranceInches
              ? feedforward.getKs() * Math.signum(errorInches)
              : 0.0;
    }
    lastSetpointVelocity = setpoint.velocity;

    double forward = pidOutput + feedforwardOutput;
    double correction =
        headingKp * (drivetrain.getHeadingDegrees() - startHeadingDegrees);
    drivetrain.tankDrive(forward + correction, forward - correction);

    SmartDashboard.putNumber("Auto/Setpoint inches", setpoint.position);
    SmartDashboard.putNumber("Auto/Setpoint in per sec", setpoint.velocity);
    SmartDashboard.putNumber("Auto/Traveled inches", traveledInches);
  }

  @Override
  public boolean isFinished() {
    return distancePid.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
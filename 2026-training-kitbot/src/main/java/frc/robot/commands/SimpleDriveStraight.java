package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveConstants;
import frc.robot.TunableNumber;
import frc.robot.subsystems.Drivetrain;

/**
 * The simple auton: drive forward at a fixed speed, stop as soon as the AVERAGE of the two encoders
 * reaches the target distance. No motion profile, no feedforward — easy to understand and debug.
 *
 * <p>While driving it keeps itself straight. If one side of the robot gets ahead of the other, it
 * slows that side down and speeds the other side up. You pick HOW it notices that:
 *
 * <ul>
 *   <li>{@link StraightMode#PIGEON} — uses the Pigeon heading. If the robot has rotated, steer
 *       back. Not fooled by wheel slip, so this is usually the better choice.
 *   <li>{@link StraightMode#ENCODERS} — compares left vs. right encoder distance. If one side has
 *       gone further, steer back. Works even if the Pigeon is broken, but a slipping wheel fools it.
 * </ul>
 */
public class SimpleDriveStraight extends Command {
  public enum StraightMode {
    PIGEON,
    ENCODERS
  }

  private final Drivetrain drivetrain;
  private final double targetInches;
  private final StraightMode mode;

  private double startLeftInches;
  private double startRightInches;
  private double startHeadingDegrees;
  private double speed;
  private double slowSpeed;
  private double slowZoneInches;
  private double headingKp;
  private double encoderKp;

  private final TunableNumber speedTunable =
      new TunableNumber("Simple auto speed", DriveConstants.SIMPLE_AUTO_SPEED);
  private final TunableNumber slowSpeedTunable =
      new TunableNumber("Simple auto slow speed", DriveConstants.SIMPLE_AUTO_SLOW_SPEED);
  private final TunableNumber slowZoneTunable =
      new TunableNumber("Simple auto slow zone inches", DriveConstants.SIMPLE_AUTO_SLOW_ZONE_INCHES);
  private final TunableNumber headingKpTunable =
      new TunableNumber("Simple auto heading kP", DriveConstants.HEADING_KP);
  private final TunableNumber encoderKpTunable =
      new TunableNumber("Simple auto encoder kP", DriveConstants.ENCODER_STRAIGHT_KP);

  public SimpleDriveStraight(Drivetrain drivetrain, double targetInches, StraightMode mode) {
    this.drivetrain = drivetrain;
    this.targetInches = targetInches;
    this.mode = mode;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Remember where we started, so we don't need to reset the sensors first.
    startLeftInches = drivetrain.getLeftInches();
    startRightInches = drivetrain.getRightInches();
    startHeadingDegrees = drivetrain.getHeadingDegrees();

    // Read the dashboard numbers once, at the start of each run.
    speed = speedTunable.get();
    slowSpeed = slowSpeedTunable.get();
    slowZoneInches = slowZoneTunable.get();
    headingKp = headingKpTunable.get();
    encoderKp = encoderKpTunable.get();
  }

  @Override
  public void execute() {
    double leftTraveled = drivetrain.getLeftInches() - startLeftInches;
    double rightTraveled = drivetrain.getRightInches() - startRightInches;
    double averageTraveled = (leftTraveled + rightTraveled) / 2.0;
    double inchesLeftToGo = targetInches - averageTraveled;
    double headingError = drivetrain.getHeadingDegrees() - startHeadingDegrees;
    double encoderError = rightTraveled - leftTraveled;

    // Drive at full auto speed, then slow down near the end so we don't coast past the line.
    double forward = inchesLeftToGo < slowZoneInches ? slowSpeed : speed;

    // Both errors are POSITIVE when the right side is ahead (robot turning left).
    // Positive correction speeds up the left side and slows the right side.
    double correction =
        mode == StraightMode.PIGEON ? headingKp * headingError : encoderKp * encoderError;
    // Never let the steering fix be bigger than the forward speed (no spinning in place).
    correction = MathUtil.clamp(correction, -forward, forward);

    drivetrain.tankDrive(forward + correction, forward - correction);

    SmartDashboard.putString("Auto/Simple mode", mode.name());
    SmartDashboard.putNumber("Auto/Simple traveled inches", averageTraveled);
    SmartDashboard.putNumber("Auto/Simple inches to go", inchesLeftToGo);
    SmartDashboard.putNumber("Auto/Simple heading error deg", headingError);
    SmartDashboard.putNumber("Auto/Simple encoder error inches", encoderError);
    SmartDashboard.putNumber("Auto/Simple correction", correction);
  }

  @Override
  public boolean isFinished() {
    double averageTraveled =
        ((drivetrain.getLeftInches() - startLeftInches)
                + (drivetrain.getRightInches() - startRightInches))
            / 2.0;
    return averageTraveled >= targetInches;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}

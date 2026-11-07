package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveConstants;
import frc.robot.TunableNumber;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

/**
 * Turns in place (left side one way, right side the other) until the Pigeon says the robot points
 * at the target heading.
 *
 * <p>The target is an ABSOLUTE heading (e.g. "-90 degrees from where auton started"), not "turn 90
 * more degrees". If an earlier turn stopped at 88 degrees, the next one still aims for exactly 180,
 * so errors don't pile up.
 *
 * <p>Tuning (all on the dashboard under "Tuning/"):
 *
 * <ul>
 *   <li><b>Turn kP</b> — turn speed per degree still to go. Raise if turns are slow, lower if it
 *       swings past the target.
 *   <li><b>Turn min output</b> — the least it will ever push. A tank drive scrubs its wheels
 *       sideways when it turns, so it needs MORE than driving straight to get moving. If the robot
 *       stops a few degrees short and just sits there, raise this.
 *   <li><b>Turn max output</b> — fastest it will turn.
 * </ul>
 */
public class TurnToHeading extends Command {
  private static final TunableNumber kP = new TunableNumber("Turn kP", DriveConstants.TURN_KP);
  private static final TunableNumber minOutput =
      new TunableNumber("Turn min output", DriveConstants.TURN_MIN_OUTPUT);
  private static final TunableNumber maxOutput =
      new TunableNumber("Turn max output", DriveConstants.TURN_MAX_OUTPUT);
  private static final TunableNumber toleranceDegrees =
      new TunableNumber("Turn tolerance deg", DriveConstants.TURN_TOLERANCE_DEGREES);

  // Must stay inside the tolerance this many loops in a row (20 ms each) to count as done, so a
  // robot swinging THROUGH the target doesn't count.
  private static final int SETTLED_LOOPS = 10;

  private final Drivetrain drivetrain;
  private final String name;
  private final DoubleSupplier targetHeadingDegrees;
  private double target;
  private double kp;
  private double min;
  private double max;
  private double tolerance;
  private int loopsInTolerance;

  public TurnToHeading(Drivetrain drivetrain, String name, DoubleSupplier targetHeadingDegrees) {
    this.drivetrain = drivetrain;
    this.name = name;
    this.targetHeadingDegrees = targetHeadingDegrees;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    target = targetHeadingDegrees.getAsDouble();
    kp = kP.get();
    min = minOutput.get();
    max = maxOutput.get();
    tolerance = toleranceDegrees.get();
    loopsInTolerance = 0;
    SmartDashboard.putString("Auto/Step", name);
  }

  /** Degrees still to turn. Positive = turn left (CCW), negative = turn right. */
  private double error() {
    return MathUtil.inputModulus(target - drivetrain.getHeadingDegrees(), -180.0, 180.0);
  }

  @Override
  public void execute() {
    double error = error();
    double turn = 0.0;
    if (Math.abs(error) > tolerance) {
      turn = MathUtil.clamp(kp * error, -max, max);
      if (Math.abs(turn) < min) {
        turn = Math.copySign(min, error);
      }
      loopsInTolerance = 0;
    } else {
      loopsInTolerance++;
    }
    // Positive turn = CCW (left): left side backward, right side forward.
    drivetrain.tankDrive(-turn, turn);
    SmartDashboard.putNumber("Auto/Turn error deg", error);
  }

  @Override
  public boolean isFinished() {
    return loopsInTolerance >= SETTLED_LOOPS;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

/**
 * One straight leg of a multi-step auton. Works like the simple auton (drive until the average
 * encoder distance reaches the target, slow down near the end), with two differences:
 *
 * <ul>
 *   <li>It holds a FIXED heading you give it (e.g. "-90 degrees from where auton started") instead
 *       of "whatever heading I had when this leg started". That way a small error after a turn
 *       gets fixed on the next leg instead of piling up.
 *   <li>If the robot gets blocked (drives into a wall) it stops trying and ends the leg, so the
 *       auton keeps going instead of pushing the wall forever.
 * </ul>
 *
 * <p>Distance is measured from where THIS leg started, so encoder drift from earlier legs or wall
 * hits doesn't matter.
 */
public class DriveLeg extends Command {
  // Blocked = barely moving for this long, after the robot has had time to get going.
  private static final double BLOCKED_SPEED_IN_PER_SEC = 3.0;
  private static final double BLOCKED_SECONDS = 0.3;
  private static final double SPEED_UP_GRACE_SECONDS = 0.5;

  private final Drivetrain drivetrain;
  private final String name;
  private final double targetInches;
  private final DoubleSupplier speed;
  private final DoubleSupplier slowSpeed;
  private final double slowZoneInches;
  private final DoubleSupplier crawlSpeed;
  private final double crawlZoneInches;
  private final DoubleSupplier targetHeadingDegrees;
  private final DoubleSupplier headingKp;

  private final Timer timer = new Timer();
  private final Timer blockedTimer = new Timer();
  private double startLeftInches;
  private double startRightInches;
  private double heading;
  private double kP;
  private boolean blocked;

  /**
   * @param name shown on the dashboard so you can see which step is running
   * @param targetInches positive = forward, negative = backward
   * @param speed main speed (0 to 1)
   * @param slowSpeed speed for the last {@code slowZoneInches} (0 to 1)
   * @param slowZoneInches how far before the end to switch to slowSpeed (0 = never slow down)
   * @param crawlSpeed speed for the very last {@code crawlZoneInches}, so it stops right on the
   *     line instead of coasting past (0 to 1)
   * @param crawlZoneInches how far before the end to switch to crawlSpeed (0 = no crawl)
   * @param targetHeadingDegrees heading to hold, same units as Drivetrain.getHeadingDegrees()
   */
  public DriveLeg(
      Drivetrain drivetrain,
      String name,
      double targetInches,
      DoubleSupplier speed,
      DoubleSupplier slowSpeed,
      double slowZoneInches,
      DoubleSupplier crawlSpeed,
      double crawlZoneInches,
      DoubleSupplier targetHeadingDegrees,
      DoubleSupplier headingKp) {
    this.drivetrain = drivetrain;
    this.name = name;
    this.targetInches = targetInches;
    this.speed = speed;
    this.slowSpeed = slowSpeed;
    this.slowZoneInches = slowZoneInches;
    this.crawlSpeed = crawlSpeed;
    this.crawlZoneInches = crawlZoneInches;
    this.targetHeadingDegrees = targetHeadingDegrees;
    this.headingKp = headingKp;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    startLeftInches = drivetrain.getLeftInches();
    startRightInches = drivetrain.getRightInches();
    heading = targetHeadingDegrees.getAsDouble();
    kP = headingKp.getAsDouble();
    blocked = false;
    timer.restart();
    blockedTimer.restart();
    SmartDashboard.putString("Auto/Step", name);
  }

  private double traveled() {
    return ((drivetrain.getLeftInches() - startLeftInches)
            + (drivetrain.getRightInches() - startRightInches))
        / 2.0;
  }

  @Override
  public void execute() {
    double direction = Math.signum(targetInches);
    double inchesLeftToGo = Math.abs(targetInches) - direction * traveled();
    // Fast, then slow near the end, then crawl for the last bit so it doesn't coast past.
    double legSpeed;
    if (inchesLeftToGo < crawlZoneInches) {
      legSpeed = crawlSpeed.getAsDouble();
    } else if (inchesLeftToGo < slowZoneInches) {
      legSpeed = slowSpeed.getAsDouble();
    } else {
      legSpeed = speed.getAsDouble();
    }
    double forward = direction * legSpeed;

    // Positive when the robot has turned LEFT of where it should point. Wrapped to -180..180 so
    // "-179" vs "+179" counts as 2 degrees off, not 358.
    double headingError =
        MathUtil.inputModulus(drivetrain.getHeadingDegrees() - heading, -180.0, 180.0);
    // Positive correction speeds up the left side = turns right, back toward the target.
    double correction = MathUtil.clamp(kP * headingError, -Math.abs(forward), Math.abs(forward));
    drivetrain.tankDrive(forward + correction, forward - correction);

    double speedNow =
        Math.abs(drivetrain.getLeftInchesPerSecond() + drivetrain.getRightInchesPerSecond()) / 2.0;
    if (timer.get() < SPEED_UP_GRACE_SECONDS || speedNow > BLOCKED_SPEED_IN_PER_SEC) {
      blockedTimer.restart();
    } else if (blockedTimer.get() > BLOCKED_SECONDS) {
      blocked = true;
    }

    SmartDashboard.putNumber("Auto/Leg traveled inches", traveled());
    SmartDashboard.putNumber("Auto/Leg inches to go", inchesLeftToGo);
    SmartDashboard.putNumber("Auto/Leg heading error deg", headingError);
  }

  @Override
  public boolean isFinished() {
    return blocked || Math.signum(targetInches) * traveled() >= Math.abs(targetInches);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    if (blocked) {
      SmartDashboard.putString(
          "Auto/Step", String.format("%s: blocked after %.1f in", name, traveled()));
    }
  }
}

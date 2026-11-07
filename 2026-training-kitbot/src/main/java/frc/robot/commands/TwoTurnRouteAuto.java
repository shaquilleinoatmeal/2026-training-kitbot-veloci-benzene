package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DriveConstants;
import frc.robot.TunableNumber;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

/**
 * Multi-step auton:
 *
 * <ol>
 *   <li>Forward 20 ft: first 15 ft fast, last 5 ft slow
 *   <li>Turn right 90 degrees
 *   <li>Forward 10 ft slow
 *   <li>Turn right another 90 degrees (now facing back the way we came)
 *   <li>Forward 20 ft fast — if it hits a wall first, it notices it's blocked and moves on
 *   <li>Back up exactly 10 ft (slow, then crawl for the last 2 ft, like the simple auton)
 * </ol>
 *
 * <p>Every heading is measured from where the robot pointed when auton STARTED (0, -90, -180), not
 * from the previous step. So if a turn ends at 88 degrees, the next leg steers itself the last 2
 * degrees. Every distance is measured from where that leg started, so a wall hit or wheel slip on
 * one leg doesn't throw off the next.
 *
 * <p>Right turn = clockwise = heading goes DOWN (WPILib: counter-clockwise is positive).
 *
 * <p>Speeds and kP are on the dashboard under "Tuning/Route ..." and apply on the next run.
 */
public class TwoTurnRouteAuto extends SequentialCommandGroup {
  private static final TunableNumber fastSpeed =
      new TunableNumber("Route fast speed", DriveConstants.ROUTE_FAST_SPEED);
  private static final TunableNumber slowSpeed =
      new TunableNumber("Route slow speed", DriveConstants.ROUTE_SLOW_SPEED);
  private static final TunableNumber crawlSpeed =
      new TunableNumber("Route crawl speed", DriveConstants.ROUTE_CRAWL_SPEED);
  private static final TunableNumber headingKp =
      new TunableNumber("Route heading kP", DriveConstants.HEADING_KP);

  private static final double FEET = 12.0;
  private static final double LEG_TIMEOUT_SECONDS = 5.0;
  private static final double TURN_TIMEOUT_SECONDS = 2.5;
  private static final double PAUSE_BEFORE_BACKUP_SECONDS = 0.2;

  // Heading when auton started. Everything else is relative to this.
  private double startHeading;

  public TwoTurnRouteAuto(Drivetrain drivetrain) {
    double crawlZone = DriveConstants.ROUTE_CRAWL_ZONE_INCHES;
    addCommands(
        Commands.runOnce(() -> startHeading = drivetrain.getHeadingDegrees()),
        new DriveLeg(
                drivetrain,
                "1: forward 20 ft (15 fast, 5 slow)",
                20 * FEET,
                fastSpeed::get,
                slowSpeed::get,
                5 * FEET,
                crawlSpeed::get,
                crawlZone,
                heading(0),
                headingKp::get)
            .withTimeout(LEG_TIMEOUT_SECONDS),
        new TurnToHeading(drivetrain, "2: turn right 90", heading(-90))
            .withTimeout(TURN_TIMEOUT_SECONDS),
        new DriveLeg(
                drivetrain,
                "3: forward 10 ft slow",
                10 * FEET,
                slowSpeed::get,
                slowSpeed::get,
                0,
                crawlSpeed::get,
                crawlZone,
                heading(-90),
                headingKp::get)
            .withTimeout(LEG_TIMEOUT_SECONDS),
        new TurnToHeading(drivetrain, "4: turn right 90 more", heading(-180))
            .withTimeout(TURN_TIMEOUT_SECONDS),
        // No slow-down here: this one is expected to hit the wall. The back-up measures from
        // wherever it actually stops, so overshooting doesn't matter.
        new DriveLeg(
                drivetrain,
                "5: forward 20 ft fast",
                20 * FEET,
                fastSpeed::get,
                fastSpeed::get,
                0,
                fastSpeed::get,
                0,
                heading(-180),
                headingKp::get)
            .withTimeout(LEG_TIMEOUT_SECONDS),
        // Let it stop (or bounce off the wall) before reversing. Distance for the back-up is
        // measured from where the robot is AFTER this pause.
        Commands.run(drivetrain::stop, drivetrain).withTimeout(PAUSE_BEFORE_BACKUP_SECONDS),
        new DriveLeg(
                drivetrain,
                "6: back up 10 ft",
                -10 * FEET,
                slowSpeed::get,
                slowSpeed::get,
                0,
                crawlSpeed::get,
                2 * FEET,
                heading(-180),
                headingKp::get)
            .withTimeout(LEG_TIMEOUT_SECONDS),
        Commands.runOnce(drivetrain::stop, drivetrain));
  }

  /** A heading this many degrees from where auton started (negative = to the right). */
  private DoubleSupplier heading(double degreesFromStart) {
    return () -> startHeading + degreesFromStart;
  }
}

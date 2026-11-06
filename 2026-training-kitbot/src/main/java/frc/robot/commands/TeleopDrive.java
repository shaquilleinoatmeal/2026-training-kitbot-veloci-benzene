package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveConstants;
import frc.robot.TunableNumber;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

/**
 * Normal arcade drive, plus "heading hold": while the driver is going only straight forward or only
 * straight backward (left stick moving, right stick centered), it uses the Pigeon to keep the robot
 * pointed where it was pointing. If one side starts pulling ahead, it gets slowed down.
 *
 * <ul>
 *   <li>Touch the right stick → the driver steers, heading hold is off.
 *   <li>Let go of the right stick → wait {@code HEADING_HOLD_SETTLE_SECONDS} for the robot to stop
 *       rotating, then lock that direction in.
 *   <li>Left stick centered (robot stopped) → no correction, so it never wiggles in place.
 * </ul>
 *
 * <p>Turn it off from the dashboard with {@code Tuning/Teleop heading hold}.
 */
public class TeleopDrive extends Command {
  private static final String ENABLED_KEY = "Tuning/Teleop heading hold";

  private final Drivetrain drivetrain;
  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier rotationSupplier;
  private final DoubleSupplier maxOutputSupplier;

  private final TunableNumber headingKp =
      new TunableNumber("Teleop heading kP", DriveConstants.TELEOP_HEADING_KP);
  private final TunableNumber maxCorrection =
      new TunableNumber("Teleop max correction", DriveConstants.TELEOP_MAX_CORRECTION);

  private final Timer sinceSteering = new Timer();
  private double targetHeadingDegrees;

  /**
   * @param forward -1..1, positive = forward (already deadbanded)
   * @param rotation -1..1, positive = turn left / CCW (already deadbanded)
   * @param maxOutput 0..1 speed cap
   */
  public TeleopDrive(
      Drivetrain drivetrain,
      DoubleSupplier forward,
      DoubleSupplier rotation,
      DoubleSupplier maxOutput) {
    this.drivetrain = drivetrain;
    this.forwardSupplier = forward;
    this.rotationSupplier = rotation;
    this.maxOutputSupplier = maxOutput;
    SmartDashboard.setDefaultBoolean(ENABLED_KEY, true);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    targetHeadingDegrees = drivetrain.getHeadingDegrees();
    sinceSteering.restart();
  }

  @Override
  public void execute() {
    double forward = forwardSupplier.getAsDouble();
    double rotation = rotationSupplier.getAsDouble();
    double heading = drivetrain.getHeadingDegrees();

    boolean steering = rotation != 0.0;
    boolean moving = forward != 0.0;
    if (steering || !moving) {
      sinceSteering.restart();
    }

    // Square the sticks for finer control at low speed (what arcadeDrive does by default). Done
    // here so the heading correction below is NOT squared — squaring would make small fixes tiny.
    forward = Math.copySign(forward * forward, forward);
    rotation = Math.copySign(rotation * rotation, rotation);

    double correction = 0.0;
    boolean holding = false;
    if (sinceSteering.get() < DriveConstants.HEADING_HOLD_SETTLE_SECONDS) {
      // Still steering, stopped, or just let go and still rotating: keep grabbing the new heading.
      targetHeadingDegrees = heading;
    } else if (SmartDashboard.getBoolean(ENABLED_KEY, true)) {
      holding = true;
      correction =
          headingCorrection(heading - targetHeadingDegrees, headingKp.get(), maxCorrection.get());
    }

    drivetrain.arcadeDrive(forward, rotation + correction, maxOutputSupplier.getAsDouble(), false);

    SmartDashboard.putBoolean("Teleop/Heading holding", holding);
    SmartDashboard.putNumber("Teleop/Heading error deg", heading - targetHeadingDegrees);
    SmartDashboard.putNumber("Teleop/Heading correction", correction);
  }

  /**
   * Rotation to add to arcade drive to fix a heading error. Heading is CCW-positive, and positive
   * arcade rotation turns CCW, so a positive error (drifted left) needs a negative (rightward) fix.
   */
  public static double headingCorrection(double errorDegrees, double kP, double max) {
    return MathUtil.clamp(-kP * errorDegrees, -max, max);
  }
}

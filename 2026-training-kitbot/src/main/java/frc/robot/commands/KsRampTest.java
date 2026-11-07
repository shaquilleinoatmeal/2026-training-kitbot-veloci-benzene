package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Finds kS: the smallest voltage that gets the robot moving.
 *
 * <p>Slowly raises the voltage on both sides (same voltage, no steering) until the wheels start
 * turning, then stops and puts that voltage on the dashboard under "Test/kS ...". Run it on the
 * carpet, not on blocks — floor friction is exactly what kS measures.
 *
 * <p>Hold the button the whole time. Let go to stop early.
 */
public class KsRampTest extends Command {
  // How fast the voltage climbs. Slower = more accurate (the robot has less time to speed up
  // before we notice it moving), but takes longer.
  private static final double RAMP_VOLTS_PER_SEC = 0.1;
  // Wheel speed that counts as "it's moving now".
  private static final double MOVING_IN_PER_SEC = 0.5;
  // Give up if it still hasn't moved by this voltage (something is wrong: brake, wiring, encoders).
  private static final double MAX_VOLTS = 3.0;

  private static double lastForwardKs = Double.NaN;
  private static double lastBackwardKs = Double.NaN;

  private final Drivetrain drivetrain;
  private final double direction;
  private double volts;
  private boolean found;

  /**
   * @param forward true to drive forward, false to drive backward. Run both and use the average.
   */
  public KsRampTest(Drivetrain drivetrain, boolean forward) {
    this.drivetrain = drivetrain;
    this.direction = forward ? 1.0 : -1.0;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    volts = 0.0;
    found = false;
    SmartDashboard.putString("Test/kS status", "ramping...");
  }

  @Override
  public void execute() {
    volts += RAMP_VOLTS_PER_SEC * 0.02;
    double output = direction * volts / DriveConstants.VOLTAGE_COMPENSATION_VOLTS;
    drivetrain.tankDrive(output, output);
    SmartDashboard.putNumber("Test/kS ramp volts", volts);

    double speed =
        Math.abs(drivetrain.getLeftInchesPerSecond() + drivetrain.getRightInchesPerSecond()) / 2.0;
    if (speed > MOVING_IN_PER_SEC) {
      found = true;
    }
  }

  @Override
  public boolean isFinished() {
    return found || volts >= MAX_VOLTS;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    if (!found) {
      SmartDashboard.putString(
          "Test/kS status",
          interrupted ? "stopped early (button let go)" : "never moved — check wiring/encoders");
      return;
    }

    if (direction > 0) {
      lastForwardKs = volts;
      SmartDashboard.putNumber("Test/kS forward volts", volts);
    } else {
      lastBackwardKs = volts;
      SmartDashboard.putNumber("Test/kS backward volts", volts);
    }
    double average = averageKs();
    SmartDashboard.putNumber("Test/kS average volts", average);
    // The kV test subtracts this, so it automatically uses the kS you just measured.
    SmartDashboard.putNumber(KvSteadyTest.KS_KEY, average);
    SmartDashboard.putString("Test/kS status", String.format("done: %.3f V", volts));
  }

  /** Average of forward and backward kS, or whichever one has been measured so far. */
  private static double averageKs() {
    if (Double.isNaN(lastForwardKs)) {
      return lastBackwardKs;
    }
    if (Double.isNaN(lastBackwardKs)) {
      return lastForwardKs;
    }
    return (lastForwardKs + lastBackwardKs) / 2.0;
  }
}

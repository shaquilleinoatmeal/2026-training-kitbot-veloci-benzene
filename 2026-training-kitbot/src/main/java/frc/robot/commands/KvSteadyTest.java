package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveConstants;
import frc.robot.TunableNumber;
import frc.robot.subsystems.Drivetrain;

/**
 * Finds kV: how many volts each inch-per-second of speed costs.
 *
 * <p>Drives both sides at one fixed voltage ("Tuning/kV test volts", default 4 V). Once the speed
 * has leveled off it averages the speed and works out kV = (volts - kS) / speed, shown live on the
 * dashboard as "Test/kV". Needs room: at 4 V the robot goes about 3 ft/s.
 *
 * <p>Hold the button to drive, let go to stop. The result stays on the dashboard after you let go.
 * Run it at 2 or 3 different voltages (e.g. 3, 4, 6 V) — the kV values should be close; use the
 * average.
 */
public class KvSteadyTest extends Command {
  static final String KS_KEY = "Tuning/kS for kV test";
  // Ignore the first part while the robot is still speeding up.
  private static final double SETTLE_SECONDS = 1.0;

  private static final TunableNumber testVolts = new TunableNumber("kV test volts", 4.0);
  private static final TunableNumber ksVolts =
      new TunableNumber("kS for kV test", DriveConstants.KS_VOLTS);

  private final Drivetrain drivetrain;
  private final double direction;
  private final Timer timer = new Timer();
  private double volts;
  private double ks;
  private double speedSum;
  private int samples;

  /**
   * @param forward true to drive forward, false to drive backward.
   */
  public KvSteadyTest(Drivetrain drivetrain, boolean forward) {
    this.drivetrain = drivetrain;
    this.direction = forward ? 1.0 : -1.0;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    volts = Math.abs(testVolts.get());
    ks = ksVolts.get();
    // Echo back what the robot actually read, so you can confirm your dashboard edits arrived.
    SmartDashboard.putNumber("Test/kV robot is using volts", volts);
    SmartDashboard.putNumber("Test/kV robot is using kS", ks);
    speedSum = 0.0;
    samples = 0;
    timer.restart();
    SmartDashboard.putString("Test/kV status", "speeding up...");
  }

  @Override
  public void execute() {
    double output = direction * volts / DriveConstants.VOLTAGE_COMPENSATION_VOLTS;
    drivetrain.tankDrive(output, output);

    double speed =
        Math.abs(drivetrain.getLeftInchesPerSecond() + drivetrain.getRightInchesPerSecond()) / 2.0;
    SmartDashboard.putNumber("Test/kV speed in per sec", speed);
    if (timer.get() < SETTLE_SECONDS) {
      return;
    }

    speedSum += speed;
    samples++;
    double averageSpeed = speedSum / samples;
    if (averageSpeed < 1.0) {
      SmartDashboard.putString("Test/kV status", "not moving — raise kV test volts");
      return;
    }
    double kV = (volts - ks) / averageSpeed;
    SmartDashboard.putNumber("Test/kV", kV);
    SmartDashboard.putNumber("Test/kV average speed", averageSpeed);
    SmartDashboard.putString(
        "Test/kV status",
        String.format("measuring at %.2f V, kS %.3f (%.1f s of data)", volts, ks, samples * 0.02));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    if (samples > 0) {
      SmartDashboard.putString(
          "Test/kV status", String.format("done at %.1f V (%.1f s of data)", volts, samples * 0.02));
    } else {
      SmartDashboard.putString("Test/kV status", "let go too early — hold at least 2 s");
    }
  }
}

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class TeleopDriveSimTest {
  private static final double DT = 0.02;
  private static final String ENABLED_KEY = "Tuning/Teleop heading hold";
  private static Drivetrain drivetrain;

  @BeforeAll
  static void setUpAll() {
    drivetrain = SimDrivetrain.get();
  }

  @BeforeEach
  void setUp() {
    SimHooks.pauseTiming();
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    SmartDashboard.putBoolean(ENABLED_KEY, true);
    // Leave the shared sim robot stopped and zeroed, since other tests expect heading 0.
    drivetrain.stop();
    step(100);
    drivetrain.resetSensors();
    SimHooks.resumeTiming();
  }

  @Test
  void correctionTurnsBackTowardTarget() {
    // Drifted left (CCW, positive error) → turn right (negative rotation), and vice versa.
    assertTrue(TeleopDrive.headingCorrection(5.0, 0.03, 0.25) < 0.0);
    assertTrue(TeleopDrive.headingCorrection(-5.0, 0.03, 0.25) > 0.0);
    assertEquals(-0.25, TeleopDrive.headingCorrection(90.0, 0.03, 0.25), 1e-9);
  }

  /** Knock the robot sideways mid-drive; with heading hold on it should steer back. */
  @ParameterizedTest
  @ValueSource(doubles = {0.6, -0.6})
  void holdsHeadingAfterBeingKnockedOffCourse(double forward) {
    double offAfterKnock = runKnockTest(forward, true);
    System.out.printf("Heading hold ON, forward %.1f: %.2f deg off at end%n", forward, offAfterKnock);
    assertEquals(0.0, offAfterKnock, 2.0);
  }

  @Test
  void withoutHoldTheKnockStays() {
    double offAfterKnock = runKnockTest(0.6, false);
    System.out.printf("Heading hold OFF: %.2f deg off at end%n", offAfterKnock);
    assertTrue(Math.abs(offAfterKnock) > 5.0, "the knock was too small to prove anything");
  }

  /** Returns how many degrees the robot ends up away from where it was pointing. */
  private double runKnockTest(double forward, boolean holdEnabled) {
    SmartDashboard.putBoolean(ENABLED_KEY, holdEnabled);
    CommandScheduler.getInstance()
        .schedule(new TeleopDrive(
                drivetrain, () -> forward, () -> 0.0, () -> DriveConstants.TELEOP_MAX_OUTPUT));

    step(50); // 1 s of straight driving; heading hold locks in after the settle time.
    double targetHeading = drivetrain.getHeadingDegrees();

    // Knock (like one wheel hitting a bump): for 0.4 s, override the motors with a turn after
    // the command sets them.
    for (int i = 0; i < 20; i++) {
      CommandScheduler.getInstance().run();
      drivetrain.tankDrive(forward + 0.2, forward - 0.2);
      SimHooks.stepTiming(DT);
    }
    double offAfterKnock = drivetrain.getHeadingDegrees() - targetHeading;
    assertTrue(Math.abs(offAfterKnock) > 5.0, "knock didn't turn the robot");

    step(100); // 2 s more of the driver only holding the left stick.
    return drivetrain.getHeadingDegrees() - targetHeading;
  }

  private static void step(int loops) {
    for (int i = 0; i < loops; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(DT);
    }
  }
}

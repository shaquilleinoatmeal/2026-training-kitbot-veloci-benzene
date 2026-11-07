package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TwoTurnRouteAuto;
import frc.robot.subsystems.Drivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class TwoTurnRouteAutoSimTest {
  private static final double DT = 0.02;
  private static Drivetrain drivetrain;

  @BeforeAll
  static void setUpAll() {
    drivetrain = SimDrivetrain.get();
  }

  @BeforeEach
  void setUp() {
    SimHooks.pauseTiming();
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    drivetrain.stop();
    step(100);
    drivetrain.resetSensors();
    step(1);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    drivetrain.stop();
    step(100);
    drivetrain.resetSensors();
    SimHooks.resumeTiming();
  }

  private static void step(int loops) {
    for (int i = 0; i < loops; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(DT);
    }
  }

  private static String poseText() {
    Pose2d pose = drivetrain.getPose();
    return String.format(
        "x %.1f in, y %.1f in, heading %.1f deg",
        Units.metersToInches(pose.getX()),
        Units.metersToInches(pose.getY()),
        drivetrain.getHeadingDegrees());
  }

  @Test
  void drivesTheWholeRoute() {
    Command auto = new TwoTurnRouteAuto(drivetrain);
    CommandScheduler.getInstance().schedule(auto);

    String lastStep = "";
    double backUpStartX = Double.NaN;
    double elapsed = 0.0;
    while (auto.isScheduled() && elapsed < 30.0) {
      step(1);
      elapsed += DT;
      String stepName = SmartDashboard.getString("Auto/Step", "");
      if (!stepName.equals(lastStep)) {
        System.out.printf("%5.2f s  start %-40s at %s%n", elapsed, stepName, poseText());
        lastStep = stepName;
        if (stepName.startsWith("6")) {
          backUpStartX = Units.metersToInches(drivetrain.getPose().getX());
        }
      }
    }
    assertFalse(auto.isScheduled(), "route never finished");
    step(50); // come to rest
    System.out.printf("%5.2f s  finished, at rest at %s%n", elapsed, poseText());

    assertTrue(elapsed < 10.0, "route must fit in the 10 s auto period, took " + elapsed);

    // Start (0,0) facing +x. 20 ft forward -> x 240. Right 90 -> facing -y. 10 ft -> y -120.
    // Right 90 -> facing -x. 20 ft fast (no wall in sim, so it coasts a bit past x 0).
    // Back up 10 ft from wherever it stopped, still facing -x.
    Pose2d pose = drivetrain.getPose();
    assertEquals(-120.0, Units.metersToInches(pose.getY()), 5.0, "second leg (y) off");
    assertEquals(
        120.0, Units.metersToInches(pose.getX()) - backUpStartX, 3.0, "back-up wasn't 10 ft");
    double headingOff = MathUtil.inputModulus(drivetrain.getHeadingDegrees() - 180.0, -180, 180);
    assertEquals(0.0, headingOff, 3.0, "should end facing back the way it came");
  }
}

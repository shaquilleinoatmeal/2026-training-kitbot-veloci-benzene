package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TurnToHeading;
import frc.robot.subsystems.Drivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/** Same turn the Test-mode D-pad runs: 90 degrees from wherever the robot points now. */
class TurnToHeadingSimTest {
  private static final double DT = 0.02;
  private static Drivetrain drivetrain;

  @BeforeAll
  static void setUpAll() {
    drivetrain = SimDrivetrain.get();
  }

  @BeforeEach
  void setUp() {
    SimHooks.pauseTiming();
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    drivetrain.stop();
    step(100);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    drivetrain.stop();
    step(100);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  private static void step(int loops) {
    for (int i = 0; i < loops; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(DT);
    }
  }

  @ParameterizedTest
  @ValueSource(doubles = {-90.0, 90.0})
  void turnsNinetyFromCurrentHeading(double degrees) {
    double start = drivetrain.getHeadingDegrees();
    Command turn =
        new TurnToHeading(drivetrain, "test", () -> drivetrain.getHeadingDegrees() + degrees);
    CommandScheduler.getInstance().schedule(turn);
    double elapsed = 0.0;
    while (turn.isScheduled() && elapsed < 5.0) {
      step(1);
      elapsed += DT;
    }
    assertFalse(turn.isScheduled(), "turn never settled");
    step(25); // let it come to rest

    double turned = MathUtil.inputModulus(drivetrain.getHeadingDegrees() - start, -180, 180);
    System.out.printf(
        "Turn %+.0f: turned %.1f deg; %s%n",
        degrees, turned, SmartDashboard.getString("Turn/Result", "?"));
    assertEquals(degrees, turned, 3.0);
    assertTrue(SmartDashboard.getString("Turn/Result", "").startsWith("done"));
    assertTrue(SmartDashboard.getNumber("Turn/Time sec", 99) < 2.0, "turn too slow");
  }
}

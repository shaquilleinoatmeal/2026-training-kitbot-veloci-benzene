package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.KsRampTest;
import frc.robot.commands.KvSteadyTest;
import frc.robot.subsystems.Drivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class FeedforwardTestsSimTest {
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
    settle();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
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

  // Let the robot come to a full stop so each test starts from rest.
  private static void settle() {
    drivetrain.stop();
    step(100);
  }

  // The sim's friction is exactly DriveConstants.KS_VOLTS, so the ramp should find about that.
  @ParameterizedTest
  @ValueSource(booleans = {true, false})
  void ksRampFindsSimFriction(boolean forward) {
    Command test = new KsRampTest(drivetrain, forward);
    CommandScheduler.getInstance().schedule(test);
    double elapsed = 0.0;
    while (test.isScheduled() && elapsed < 40.0) {
      step(1);
      elapsed += DT;
    }
    assertFalse(test.isScheduled(), "kS ramp never finished");

    String key = forward ? "Test/kS forward volts" : "Test/kS backward volts";
    double ks = SmartDashboard.getNumber(key, Double.NaN);
    System.out.printf("kS (%s) measured %.3f V, sim friction %.3f V%n",
        forward ? "forward" : "backward", ks, DriveConstants.KS_VOLTS);
    assertEquals(DriveConstants.KS_VOLTS, ks, 0.15);
    settle();
  }

  @ParameterizedTest
  @ValueSource(booleans = {true, false})
  void kvSteadyGivesPlausibleKv(boolean forward) {
    SmartDashboard.putNumber("Tuning/kS for kV test", DriveConstants.KS_VOLTS);
    Command test = new KvSteadyTest(drivetrain, forward);
    CommandScheduler.getInstance().schedule(test);
    step(150); // hold the button 3 s
    test.cancel();
    step(1);

    double kV = SmartDashboard.getNumber("Test/kV", Double.NaN);
    System.out.printf("kV (%s) measured %.4f V per in/s%n", forward ? "forward" : "backward", kV);
    // CIM kitbot on 6" wheels, 10.71:1: roughly 0.07-0.09 V per in/s.
    assertTrue(kV > 0.06 && kV < 0.11, "kV out of plausible range: " + kV);
    settle();
  }
}

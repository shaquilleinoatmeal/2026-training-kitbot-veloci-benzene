package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/** Picks each auton on the "Auto" chooser the way the dashboard does, then runs it. */
class AutoChooserSimTest {
  private static final double DT = 0.02;
  private static Drivetrain drivetrain;
  private static RobotContainer container;

  @BeforeAll
  static void setUpAll() {
    drivetrain = SimDrivetrain.get();
    container = new RobotContainer(drivetrain);
  }

  @BeforeEach
  void setUp() {
    SimHooks.pauseTiming();
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    drivetrain.stop();
    step(100);
    drivetrain.resetSensors();
    SimHooks.resumeTiming();
  }

  @ParameterizedTest
  @ValueSource(
      strings = {
        "Simple 10 ft (Pigeon straight)",
        "Simple 10 ft (encoder straight)",
        "Drive 10 ft"
      })
  void chosenAutonDrivesTenFeet(String option) {
    // This is exactly what clicking an option on the dashboard does.
    NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getSubTable("Auto")
        .getEntry("selected")
        .setString(option);
    SmartDashboard.updateValues();

    double startInches = drivetrain.getAverageInches();
    Command auto = container.getAutonomousCommand();
    assertNotNull(auto);
    SmartDashboard.updateValues(); // the dashboard's "active" field refreshes here
    assertEquals(option, chosenName(), "chooser didn't pick up the selection");
    CommandScheduler.getInstance().schedule(auto);

    double elapsed = 0.0;
    while (auto.isScheduled() && elapsed < Math.max(DriveConstants.AUTO_TIMEOUT_SECONDS, DriveConstants.SIMPLE_AUTO_TIMEOUT_SECONDS) + 0.5) {
      step(1);
      elapsed += DT;
    }
    assertFalse(auto.isScheduled(), option + " never finished");
    step(50); // coast to a stop

    double traveled = drivetrain.getAverageInches() - startInches;
    System.out.printf("Chooser '%s': drove %.2f in in %.2f s%n", option, traveled, elapsed);
    assertEquals(DriveConstants.AUTO_DISTANCE_INCHES, traveled, 6.0);
  }

  private static String chosenName() {
    return NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getSubTable("Auto")
        .getEntry("active")
        .getString("");
  }

  private static void step(int loops) {
    for (int i = 0; i < loops; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(DT);
    }
  }
}

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SimpleDriveStraight;
import frc.robot.commands.SimpleDriveStraight.StraightMode;
import frc.robot.subsystems.Drivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

class SimpleDriveStraightSimTest {
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
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    SimHooks.resumeTiming();
  }

  @ParameterizedTest
  @EnumSource(StraightMode.class)
  void drivesTenFeetAndStops(StraightMode mode) {
    double startInches = drivetrain.getAverageInches();
    double startHeading = drivetrain.getHeadingDegrees();
    Command auto = new SimpleDriveStraight(drivetrain, DriveConstants.AUTO_DISTANCE_INCHES, mode);
    CommandScheduler.getInstance().schedule(auto);

    double elapsed = 0.0;
    while (auto.isScheduled() && elapsed < DriveConstants.SIMPLE_AUTO_TIMEOUT_SECONDS) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(DT);
      elapsed += DT;
    }
    assertFalse(auto.isScheduled(), "simple auton never reached 10 ft before the timeout");

    // Let the robot coast to a stop (brake mode on the real robot stops it faster than sim).
    for (int i = 0; i < 50; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(DT);
    }

    double traveled = drivetrain.getAverageInches() - startInches;
    System.out.printf(
        "Simple auton (%s) finished in %.2f s, came to rest at %.2f in, heading %.2f deg%n",
        mode, elapsed, traveled, drivetrain.getHeadingDegrees());
    assertEquals(DriveConstants.AUTO_DISTANCE_INCHES, traveled, 6.0);
    assertEquals(startHeading, drivetrain.getHeadingDegrees(), 2.0);
  }
}

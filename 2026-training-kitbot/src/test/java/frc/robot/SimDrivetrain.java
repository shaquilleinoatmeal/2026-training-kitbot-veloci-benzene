package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.Drivetrain;

/** One shared sim Drivetrain for all tests (motor/encoder IDs can only be claimed once). */
final class SimDrivetrain {
  private static Drivetrain drivetrain;

  private SimDrivetrain() {}

  static synchronized Drivetrain get() {
    if (drivetrain == null) {
      assertTrue(HAL.initialize(500, 0));
      drivetrain = new Drivetrain();
    }
    return drivetrain;
  }
}

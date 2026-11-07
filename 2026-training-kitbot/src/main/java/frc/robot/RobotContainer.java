package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.KsRampTest;
import frc.robot.commands.KvSteadyTest;
import frc.robot.commands.SimpleDriveStraight;
import frc.robot.commands.SimpleDriveStraight.StraightMode;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TwoTurnRouteAuto;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final Drivetrain drivetrain;
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SlewRateLimiter forwardLimiter =
      new SlewRateLimiter(1.0 / DriveConstants.TELEOP_RAMP_SECONDS);
  private final SlewRateLimiter rotationLimiter =
      new SlewRateLimiter(1.0 / DriveConstants.TELEOP_RAMP_SECONDS);
  private final SlewRateLimiter maxOutputLimiter =
      new SlewRateLimiter(1.0 / DriveConstants.TELEOP_RAMP_SECONDS);
  private final TunableNumber teleopMaxOutput =
      new TunableNumber("Teleop max output", DriveConstants.TELEOP_MAX_OUTPUT);

  public RobotContainer() {
    this(new Drivetrain());
  }

  // Lets tests hand in the shared simulated drivetrain (motor IDs can only be claimed once).
  RobotContainer(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    configureBindings();
    configureAutos();
  }

  private void configureBindings() {
    // Standard WPILib signs: stick Y is negative when pushed forward, and arcadeDrive turns left
    // (CCW) for positive rotation, so both sticks are negated. Motor direction is handled in
    // Drivetrain (DriveConstants.DRIVE_REVERSED), not here.
    // TeleopDrive also keeps the robot straight when only the left stick is used (heading hold).
    drivetrain.setDefaultCommand(
        new TeleopDrive(
            drivetrain,
            () ->
                forwardLimiter.calculate(
                    MathUtil.applyDeadband(-driverController.getLeftY(), DriveConstants.DEADBAND)),
            () ->
                rotationLimiter.calculate(
                    MathUtil.applyDeadband(-driverController.getRightX(), DriveConstants.DEADBAND)),
            () ->
                maxOutputLimiter.calculate(
                    driverController.getHID().getRightBumperButton()
                        ? 1.0
                        : teleopMaxOutput.get())));

    SmartDashboard.putData(
        "Reset sensors", Commands.runOnce(drivetrain::resetSensors).ignoringDisable(true));

    // Feedforward tuning — only works in Test mode on the Driver Station. HOLD the button.
    // Results show up on the dashboard under "Test/". Copy them into DriveConstants.
    //   A = kS forward, B = kS backward  (slowly ramps voltage until the robot starts moving)
    //   X = kV forward, Y = kV backward  (drives at "Tuning/kV test volts", needs room!)
    Trigger testMode = new Trigger(DriverStation::isTest);
    driverController.a().and(testMode).whileTrue(new KsRampTest(drivetrain, true));
    driverController.b().and(testMode).whileTrue(new KsRampTest(drivetrain, false));
    driverController.x().and(testMode).whileTrue(new KvSteadyTest(drivetrain, true));
    driverController.y().and(testMode).whileTrue(new KvSteadyTest(drivetrain, false));
  }

  // Pick the auton on the dashboard under "Auto" BEFORE enabling.
  // To add a new auton: write the command, then add one more addOption(...) line here.
  private void configureAutos() {
    autoChooser.setDefaultOption(
        "Drive 10 ft",
        // Positive = forward, negative = backward.
        new DriveDistance(drivetrain, DriveConstants.AUTO_DISTANCE_INCHES)
            .withTimeout(DriveConstants.AUTO_TIMEOUT_SECONDS));
    autoChooser.addOption(
        "Simple 10 ft (Pigeon straight)",
        new SimpleDriveStraight(drivetrain, DriveConstants.AUTO_DISTANCE_INCHES, StraightMode.PIGEON)
            .withTimeout(DriveConstants.SIMPLE_AUTO_TIMEOUT_SECONDS));
    autoChooser.addOption(
        "Simple 10 ft (encoder straight)",
        new SimpleDriveStraight(
                drivetrain, DriveConstants.AUTO_DISTANCE_INCHES, StraightMode.ENCODERS)
            .withTimeout(DriveConstants.SIMPLE_AUTO_TIMEOUT_SECONDS));
    autoChooser.addOption(
        "Two-turn route (20 ft, right, 10 ft, right, 20 ft, back 10 ft)",
        new TwoTurnRouteAuto(drivetrain));
    autoChooser.addOption("Do nothing", Commands.none());
    SmartDashboard.putData("Auto", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

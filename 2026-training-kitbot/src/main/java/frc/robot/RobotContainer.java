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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.SimpleDriveStraight;
import frc.robot.commands.SimpleDriveStraight.StraightMode;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
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
    configureBindings();
    configureAutos();
  }

  private void configureBindings() {
    double turnSign = DriveConstants.INVERT_TURN ? -1.0 : 1.0;
    drivetrain.setDefaultCommand(
        Commands.run(
            () ->
                drivetrain.arcadeDrive(
                    forwardLimiter.calculate(
                        MathUtil.applyDeadband(-driverController.getLeftY(), DriveConstants.DEADBAND)),
                    rotationLimiter.calculate(
                        turnSign * MathUtil.applyDeadband(-driverController.getRightX(), DriveConstants.DEADBAND)),
                    maxOutputLimiter.calculate(
                        driverController.getHID().getRightBumperButton()
                            ? 1.0
                            : teleopMaxOutput.get())),
            drivetrain));

    SmartDashboard.putData(
        "Reset sensors", Commands.runOnce(drivetrain::resetSensors).ignoringDisable(true));

    Trigger testMode = new Trigger(DriverStation::isTest);
    driverController
        .a()
        .and(testMode)
        .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driverController
        .b()
        .and(testMode)
        .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driverController
        .x()
        .and(testMode)
        .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driverController
        .y()
        .and(testMode)
        .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  // Pick the auton on the dashboard under "Auto" BEFORE enabling.
  // To add a new auton: write the command, then add one more addOption(...) line here.
  private void configureAutos() {
    autoChooser.setDefaultOption(
        "Simple 10 ft (Pigeon straight)",
        new SimpleDriveStraight(
                drivetrain, DriveConstants.AUTO_DISTANCE_INCHES, StraightMode.PIGEON)
            .withTimeout(DriveConstants.AUTO_TIMEOUT_SECONDS));
    autoChooser.addOption(
        "Simple 10 ft (encoder straight)",
        new SimpleDriveStraight(
                drivetrain, DriveConstants.AUTO_DISTANCE_INCHES, StraightMode.ENCODERS)
            .withTimeout(DriveConstants.AUTO_TIMEOUT_SECONDS));
    autoChooser.addOption(
        "Profiled 10 ft (old)",
        new DriveDistance(drivetrain, DriveConstants.AUTO_DISTANCE_INCHES)
            .withTimeout(DriveConstants.AUTO_TIMEOUT_SECONDS));
    autoChooser.addOption("Do nothing", Commands.none());
    SmartDashboard.putData("Auto", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

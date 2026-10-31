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

  private void configureAutos() {
    autoChooser.setDefaultOption(
        "Drive 10 ft",
        new DriveDistance(drivetrain, 120.0).withTimeout(DriveConstants.AUTO_TIMEOUT_SECONDS));
    autoChooser.addOption("Do nothing", Commands.none());
    SmartDashboard.putData("Auto", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

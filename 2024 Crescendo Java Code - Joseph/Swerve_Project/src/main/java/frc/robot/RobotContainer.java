// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveDrive swerveDrive = new SwerveDrive();
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController joystick0 =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveDrive.setDefaultCommand(new SwerveJoystickCmd(swerveDrive, () -> joystick0.getLeftX(),
     () -> joystick0.getLeftY(), () -> joystick0.getRightX()));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    joystick0.y().onTrue(Commands.runOnce(() -> swerveDrive.zeroGyro(), swerveDrive));
    joystick0.x().onTrue(Commands.runOnce(() -> swerveDrive.resetOdometer(), swerveDrive));
    joystick0.a().toggleOnTrue(Commands.runOnce(() -> swerveDrive.toggleIsFieldOriented(), swerveDrive));
    joystick0.rightBumper().onTrue(Commands.runOnce(() -> swerveDrive.resetOdometer(), swerveDrive));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.DriveForward;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.OnTheFlyTrajectory;
import java.util.List;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.BooleanSupplier;

import com.fasterxml.jackson.databind.ser.std.NumberSerializers.FloatSerializer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Limelight limeLight = new Limelight();
  private final SwerveDrive swerveDrive = new SwerveDrive(limeLight);
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController joystick0 =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveDrive.setDefaultCommand(new SwerveJoystickCmd(swerveDrive, () -> joystick0.getLeftX(),
     () -> joystick0.getLeftY(), () -> joystick0.getRightX()));

    chooser.setDefaultOption("Nothing", null);
    chooser.addOption("tag1", new OnTheFlyTrajectory(swerveDrive,1, () -> swerveDrive.getPose()));
    chooser.addOption("tag3", new OnTheFlyTrajectory(swerveDrive,3, () -> swerveDrive.getPose()));
		chooser.addOption("tag5", new OnTheFlyTrajectory(swerveDrive,5, () -> swerveDrive.getPose()));
    chooser.addOption("tag6", new OnTheFlyTrajectory(swerveDrive,6, () -> swerveDrive.getPose()));
    chooser.addOption("tag7", new OnTheFlyTrajectory(swerveDrive,7, () -> swerveDrive.getPose()));

    SmartDashboard.putData("Tag Choices", chooser);
    SmartDashboard.putData("Tag 1", new OnTheFlyTrajectory(swerveDrive,1, () -> swerveDrive.getPose()));
    SmartDashboard.putData("Tag 3", new OnTheFlyTrajectory(swerveDrive,3, () -> swerveDrive.getPose()));
    SmartDashboard.putData("Tag 5", new OnTheFlyTrajectory(swerveDrive,5, () -> swerveDrive.getPose()));
    SmartDashboard.putData("Tag 6", new OnTheFlyTrajectory(swerveDrive,6, () -> swerveDrive.getPose()));
    SmartDashboard.putData("Tag 7", new OnTheFlyTrajectory(swerveDrive,7, () -> swerveDrive.getPose()));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    joystick0.y().onTrue(Commands.runOnce(() -> swerveDrive.zeroGyro(), swerveDrive));
    joystick0.x().onTrue(Commands.runOnce(() -> swerveDrive.resetOdometer(), swerveDrive));
    joystick0.a().toggleOnTrue(Commands.runOnce(() -> swerveDrive.toggleIsFieldOriented(), swerveDrive));
    joystick0.b().whileTrue(new DriveForward(swerveDrive, .2));
    joystick0.leftBumper().whileTrue(new DriveForward(swerveDrive, -.2));
    joystick0.rightBumper().onTrue(Commands.runOnce(() -> swerveDrive.resetOdometer(), swerveDrive));
    joystick0.back().onTrue(new FollowTrajectory(swerveDrive, List.of(new Translation2d(1, 0),new Translation2d(1, -1)), new Pose2d(0, 0,
    Rotation2d.fromDegrees(0)),new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
    AutoConstants.trajectoryXControllerkP,AutoConstants.trajectoryYControllerkP, AutoConstants.trajectoryThetaControllerkP).getCommandSequence().andThen(new FollowTrajectory(swerveDrive, List.of(new Translation2d(-1, 0),new Translation2d(-1, 1)), new Pose2d(0, 0,
    Rotation2d.fromDegrees(180)),new Pose2d(-2, 1, Rotation2d.fromDegrees(0)),
    AutoConstants.trajectoryXControllerkP,AutoConstants.trajectoryYControllerkP, AutoConstants.trajectoryThetaControllerkP).getCommandSequence()));
    joystick0.start().onTrue(new OnTheFlyTrajectory(swerveDrive,7, () -> swerveDrive.getPose()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
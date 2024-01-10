package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import java.util.ArrayList;
import frc.robot.commands.SwerveControllerCommand;

public class FollowTrajectory{
  
  private final SwerveDrive swerveDrive;

  private final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    AutoConstants.maxDriveSpeedMetersPerSecond,
    AutoConstants.maxAccelerationMetersPerSecondSquared)
            .setKinematics(AutoConstants.kDriveKinematics);
    
  private final Trajectory trajectory;
  private final SwerveControllerCommand swerveControllerCommand;
  private final PIDController xController, yController;
  private final ProfiledPIDController thetaController;

  private final SequentialCommandGroup followTrajectoryCommandGroup;
  
  //Ideally accepts an array of Trajectory points and trajectory config and generates trajectory inside of function
  public FollowTrajectory(SwerveDrive swerveDrive, List<Translation2d> trajectoryPoints, Pose2d startPose, Pose2d endPose, 
  double trajectoryXControllerkP, double trajectoryYControllerkP, double trajectoryThetaControllerkP){
    this.swerveDrive = swerveDrive;

    //PID Controllers
    xController = new PIDController(trajectoryXControllerkP, 0, 0);
    yController = new PIDController(trajectoryYControllerkP, 0, 0);
    thetaController = new ProfiledPIDController(trajectoryThetaControllerkP, 0, 0, 
    AutoConstants.thetaTrapezoidProfile);

    //Generates Trajectory
    trajectory = TrajectoryGenerator.generateTrajectory(startPose, trajectoryPoints,
    endPose, trajectoryConfig);

    //Initializes swerve Controller Command for execution
    swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveDrive::getRelativePose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        swerveDrive::setModuleStates,
        swerveDrive);

    //Create the command group for this auto instance
    followTrajectoryCommandGroup = new SequentialCommandGroup(new InstantCommand(() -> resetLoops()), swerveControllerCommand, 
    new InstantCommand(() -> swerveDrive.stop()));
    followTrajectoryCommandGroup.addRequirements(swerveDrive);
  }

  public Command getCommandSequence(){
    return new SequentialCommandGroup(new InstantCommand(() -> swerveDrive.setRelativePose(swerveDrive.getPose())), 
    new ParallelDeadlineGroup(followTrajectoryCommandGroup, 
    new RepeatCommand(new InstantCommand(() -> swerveDrive.printRelativePoseComponents()))));
  }

  public void resetLoops(){
    xController.reset();
    yController.reset();
    thetaController.reset(swerveDrive.getPose().getRotation().getRadians());;
  }
}

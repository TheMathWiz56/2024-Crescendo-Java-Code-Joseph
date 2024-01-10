package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeLightConstants;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.CommandUtil;

import java.util.List;
import java.util.ArrayList;
import frc.robot.commands.SwerveControllerCommand;

public class OnTheFlyTrajectory extends CommandBase{
  
  private final SwerveDrive swerveDrive;

  private final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    AutoConstants.maxDriveSpeedMetersPerSecond,
    AutoConstants.maxAccelerationMetersPerSecondSquared)
            .setKinematics(AutoConstants.kDriveKinematics);
    
  private final PIDController xController, yController;
  private final ProfiledPIDController thetaController;
  private  int target;
  private final Supplier<Pose2d> robotPose;
  
  //Ideally accepts an array of Trajectory points and trajectory config and generates trajectory inside of function
  public OnTheFlyTrajectory(SwerveDrive swerveDrive, int target, Supplier<Pose2d> robotPose){
    this.swerveDrive = swerveDrive;
    this.target = target;
    this.robotPose = robotPose;

    xController = new PIDController(AutoConstants.trajectoryXControllerkP, 0, 0);
    yController = new PIDController(AutoConstants.trajectoryYControllerkP, 0, 0);
    thetaController = new ProfiledPIDController(AutoConstants.trajectoryThetaControllerkP, 0, 0, 
    AutoConstants.thetaTrapezoidProfile);
  }

  @Override 
  public void initialize(){
  }

  @Override
  public void execute(){
    Pose2d startPose = robotPose.get();
    
    SmartDashboard.putString("OnTheFlyStartPose", startPose.getTranslation().toString());

    //Generate trajectoryPoints and endPose
    Pose2d endPose = new Pose2d();
    List<Translation2d> trajectoryPoints = new ArrayList<Translation2d>();

    double offsetFromTag = 1;
    switch(target){
      case 1: 
      trajectoryPoints.add(new Translation2d(startPose.getX(), LimeLightConstants.tag1XPosition.getY()));
      endPose = new Pose2d(LimeLightConstants.tag1XPosition.plus(new Translation2d(offsetFromTag,0)), Rotation2d.fromDegrees(180));
      break;
      case 3:
      trajectoryPoints.add(new Translation2d(LimeLightConstants.tag3XPosition.getX()-offsetFromTag, startPose.getY()));
      endPose = new Pose2d(LimeLightConstants.tag3XPosition.plus(new Translation2d(-offsetFromTag,0)), new Rotation2d());
      break;
      case 5:
      trajectoryPoints.add(new Translation2d(LimeLightConstants.tag5XPosition.getX()-offsetFromTag, startPose.getY()));
      endPose = new Pose2d(LimeLightConstants.tag5XPosition.plus(new Translation2d(-offsetFromTag,0)), new Rotation2d());
      break;
      case 6:
      trajectoryPoints.add(new Translation2d(LimeLightConstants.tag6XPosition.getX()-offsetFromTag, startPose.getY()));
      endPose = new Pose2d(LimeLightConstants.tag6XPosition.plus(new Translation2d(-offsetFromTag,0)), new Rotation2d());
      break;
      case 7:
      trajectoryPoints.add(new Translation2d(startPose.getX(), LimeLightConstants.tag7XPosition.getY()));
      endPose = new Pose2d(LimeLightConstants.tag7XPosition.plus(new Translation2d(offsetFromTag,0)), Rotation2d.fromDegrees(180));
      break;
      default:
      break;
    }

    SmartDashboard.putString("Target Location", endPose.getTranslation().toString());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, trajectoryPoints,
    endPose, trajectoryConfig);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveDrive::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        swerveDrive::setModuleStates,
        swerveDrive);
    //dont not need to set relative pose in this case
    new ParallelDeadlineGroup(new SequentialCommandGroup(new InstantCommand(() -> resetLoops()), swerveControllerCommand, new InstantCommand(() -> swerveDrive.stop())), 
    new RepeatCommand(new InstantCommand(() -> swerveDrive.printRelativePoseComponents()))).schedule();
    
  }

  @Override
  public void end(boolean interrupted) {

  }

@Override
public boolean isFinished() {
    return true;
}

  public void resetLoops(){
    xController.reset();
    yController.reset();
    thetaController.reset(swerveDrive.getPose().getRotation().getRadians());;
  }

}

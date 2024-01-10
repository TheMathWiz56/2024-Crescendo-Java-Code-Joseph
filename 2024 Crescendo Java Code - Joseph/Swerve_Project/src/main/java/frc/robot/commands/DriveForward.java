package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

public class DriveForward extends CommandBase {
  
  private final SwerveDrive swerveDrive;
  private double speed;
  
  public DriveForward(SwerveDrive swerveDrive, double speed){
    this.swerveDrive = swerveDrive;
    this.speed = speed;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute(){

    double xSpeed = speed;
    double ySpeed = 0;
    double turningSpeed = 0;
    double radius = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);

    //SmartDashboard.putNumber("Radius", radius);
    if (radius < .1){
      xSpeed = 0;
      ySpeed = 0;
    }

    if (Math.abs(turningSpeed) < .1){
      turningSpeed = 0;
    }

    xSpeed *= DriveConstants.maxDriveSpeedMetersPerSecond;
    ySpeed *= DriveConstants.maxDriveSpeedMetersPerSecond;
    turningSpeed *= DriveConstants.maxAngularSpeedRadiansPerSecond;

    turningSpeed = Math.abs(turningSpeed) > OperatorConstants.kDeadband ? turningSpeed : 0.0;

    SmartDashboard.putNumber("X", xSpeed);
    SmartDashboard.putNumber("Y", ySpeed);
    SmartDashboard.putNumber("Turn", turningSpeed);

    ChassisSpeeds chassisSpeeds;
    //SmartDashboard.putBoolean("A", fieldOrientedFunction.get());
        if (swerveDrive.getIsFieldOriented()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveDrive.getRotation2d());
            SmartDashboard.putBoolean("Is Field Oriented", true);
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            SmartDashboard.putBoolean("Is Field Oriented", false);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveDrive.setModuleStates(moduleStates); // should be moduleStates
  }

  @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

  @Override
  public boolean isFinished() {
      return false;
  }
}

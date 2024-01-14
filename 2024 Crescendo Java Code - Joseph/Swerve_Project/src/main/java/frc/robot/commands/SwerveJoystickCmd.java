package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

public class SwerveJoystickCmd extends CommandBase {
  
  private final SwerveDrive swerveDrive;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  
  public SwerveJoystickCmd(SwerveDrive swerveDrive, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
  Supplier<Double> turningSpdFunction){
    this.swerveDrive = swerveDrive;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute(){

    double xSpeed = ySpdFunction.get();
    double ySpeed = xSpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
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
        if (swerveDrive.getIsFieldOriented()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveDrive.getRotation2d());
            SmartDashboard.putNumber("field xspeed", xSpeed);
            SmartDashboard.putNumber("field yspeed", ySpeed);
            SmartDashboard.putNumber("field turningspeed", turningSpeed);
            SmartDashboard.putString("field heading", swerveDrive.getRotation2d().toString());
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

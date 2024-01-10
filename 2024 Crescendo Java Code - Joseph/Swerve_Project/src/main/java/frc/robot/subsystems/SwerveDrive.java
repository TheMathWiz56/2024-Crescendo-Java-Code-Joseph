package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class SwerveDrive extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.FLDriveID, DriveConstants.FLTurnID, DriveConstants.FLDriveMotorReversed,
   DriveConstants.FLTurnMotorReversed, DriveConstants.FLCANCoderID, DriveConstants.FLOffset);
  private final SwerveModule frontRight = new SwerveModule(DriveConstants.FRDriveID, DriveConstants.FRTurnID, DriveConstants.FRDriveMotorReversed,
  DriveConstants.FRTurnMotorReversed, DriveConstants.FRCANCoderID, DriveConstants.FROffset);
  private final SwerveModule backLeft = new SwerveModule(DriveConstants.BLDriveID, DriveConstants.BLTurnID, DriveConstants.BLDriveMotorReversed,
  DriveConstants.BLTurnMotorReversed, DriveConstants.BLCANCoderID, DriveConstants.BLOffset);
  private final SwerveModule backRight = new SwerveModule(DriveConstants.BRDriveID, DriveConstants.BRTurnID, DriveConstants.BRDriveMotorReversed,
  DriveConstants.BRTurnMotorReversed, DriveConstants.BRCANCoderID, DriveConstants.BROffset);
  

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private Rotation2d yawOffset = new Rotation2d();
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
   getRotation2d(), getSwerveModulePositions());

  private boolean isFieldOriented;
  private SwerveModulePosition[] modulePositions;
  private double moduleSpeeds[] = {0};

  private Pose2d relativePose = new Pose2d();

  public SwerveDrive() {
    isFieldOriented = true;
    zeroGyro();
  }

  public double getHeading(){
    return gyro.getAngle() * -1 + yawOffset.getDegrees();
  }

  public double getRawHeading(){
    return gyro.getAngle() * -1;
  }

  public Rotation2d getRawRotation2d(){
    return Rotation2d.fromDegrees(getRawHeading());
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public SwerveModulePosition[] getSwerveModulePositions(){
    return new SwerveModulePosition[]{
      frontLeft.getSwerveModulePosition(),
      frontRight.getSwerveModulePosition(),
      backLeft.getSwerveModulePosition(),
      backRight.getSwerveModulePosition()
     };
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxDriveSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void stop(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void zeroGyro(){
    gyro.reset();
  }

  public void setRelativePose(Pose2d pose2d){
    relativePose = pose2d;
  }

  public void resetOdometer(){
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();   
    odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {new SwerveModulePosition(),new SwerveModulePosition(),
      new SwerveModulePosition(),new SwerveModulePosition()}, new Pose2d(new Translation2d(), getRotation2d()));
  }

  public void setOdometer(Pose2d pose){
    odometer.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
  }

  public Pose2d getPose(){
    Pose2d pose = odometer.getPoseMeters();
    return new Pose2d(new Translation2d(pose.getTranslation().getX(), pose.getTranslation().getY()), pose.getRotation());
  }

  public Pose2d getRelativePose(){
    Pose2d pose = odometer.getPoseMeters();
    return new Pose2d(new Translation2d(pose.getTranslation().getX(), pose.getTranslation().getY()).minus(relativePose.getTranslation()), pose.getRotation());
  }

  public void printPoseComponents(){
    SmartDashboard.putNumber("Current X", getPose().getX());
    SmartDashboard.putNumber("Current Y", getPose().getY());
  }

  public void printRelativePoseComponents(){
    SmartDashboard.putNumber("Current Relative X", getRelativePose().getX());
    SmartDashboard.putNumber("Current Relative Y", getRelativePose().getY());
  }

  public void toggleIsFieldOriented(){
    isFieldOriented = !isFieldOriented;
  }

  public boolean getIsFieldOriented(){
    return isFieldOriented;
  }

  public double[] getModuleSpeeds(){
    return new double[] {frontLeft.getDriveVelocityRPM(), frontRight.getDriveVelocityRPM(), backLeft.getDriveVelocityRPM(), backRight.getDriveVelocityRPM()};
  }
  
  public boolean isStopped(){
    moduleSpeeds = getModuleSpeeds();
    for(int i = 0; i < moduleSpeeds.length; i++){
      if (Math.abs(moduleSpeeds[i]) > DriveConstants.isStoppedThreshold){
        return false;
      }
    }
    return true;
  }

  @Override
  public void periodic(){
    modulePositions = getSwerveModulePositions();
    odometer.update(getRotation2d(), modulePositions);
    
    SmartDashboard.putNumber("Gyro Readout", getHeading());
    SmartDashboard.putString("Robot Heading", getPose().getRotation().toString());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }
}
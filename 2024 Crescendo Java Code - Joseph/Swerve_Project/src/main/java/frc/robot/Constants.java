package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.1;
  }

  public static final class DriveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(26);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(26);
        // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double maxDriveSpeedMetersPerSecond = 3.81; //3.81
    public static final double maxAccelerationMetersPerSecondSquared = 3; //3
    public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double maxAngularAccelerationRadiansPerSecondSquared = Math.PI/4;
    public static final double driveWheelCircumference = Math.PI * 4 * .0254;
    public static final double driveGearRatio = 8.14;

    public static final double RtoMeterConversion = 0.0392120164;//R / driveGearRatio * driveWheelCircumference
    public static final double RPMtoMetersPerSecondConversion = 0.0006535336069; // RPM /60 / driveGearRatio * driveWheelCircumference
    public static final double MetersPerSecondtoRPMConversion = 1530.143193; // MPS * 60 / driveWheelcircumference * driveGearRatio 

    public static final int FLDriveID = 10;
    public static final int FLTurnID = 11;
    public static final int FLCANCoderID = 20;
    public static final double FLOffset = 45;
    public static final boolean FLDriveMotorReversed = true;
    public static final boolean FLTurnMotorReversed = false;

    public static final int FRDriveID = 12;
    public static final int FRTurnID = 13;
    public static final int FRCANCoderID = 21;
    public static final double FROffset = 59.41404;
    public static final boolean FRDriveMotorReversed = true;
    public static final boolean FRTurnMotorReversed = false;

    public static final int BLDriveID = 16;
    public static final int BLTurnID = 17;
    public static final int BLCANCoderID = 23;
    public static final double BLOffset = -12.3048;
    public static final boolean BLDriveMotorReversed = true;
    public static final boolean BLTurnMotorReversed = false;

    public static final int BRDriveID = 14;
    public static final int BRTurnID = 15;
    public static final int BRCANCoderID = 22;
    public static final double BROffset = 156.53304;
    public static final boolean BRDriveMotorReversed = true;
    public static final boolean BRTurnMotorReversed = false;


    public static final double TurningPIDPeriod = .02;
    public static final double kPTurningPID = 0.00925;
    public static final double kITurningPID = 0;
    public static final double kDTurningPID = .0001;
    public static final int TurningPIDMinimum = -180;
    public static final int TurningPIDMaximum = 180;

    public static final double DrivePIDPeriod = .01;
    public static final double kPDrivePID = 0.00015;
    public static final double kIDrivePID = 0.002;
    public static final double kDDrivePID = 0;

    public static final double isStoppedThreshold = 25;
  }
}

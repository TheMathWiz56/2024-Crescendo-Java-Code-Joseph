package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import javax.xml.stream.events.StartElement;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;



public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final boolean driveMotorReversed;
    private final RelativeEncoder driveEncoder;
    private final CANSparkMax turnMotor;
    private final boolean turnMotorReversed;

    
    private final CANcoder canCoder;
    private final double canCoderOffset;

    private final PIDController turnPIDController;
    private final PIDController drivePIDController;

    public SwerveModule(int driveMotorid, int turnMotorid, boolean driveMotorReversed, boolean turnMotorReversed, int canCoderid, double canCoderOffset){

        driveMotor = new CANSparkMax(driveMotorid, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        this.driveMotorReversed = driveMotorReversed;
        turnMotor = new CANSparkMax(turnMotorid, MotorType.kBrushless);
        this.turnMotorReversed = turnMotorReversed;

        canCoder = new CANcoder(canCoderid);
        this.canCoderOffset = canCoderOffset;

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        turnPIDController = new PIDController(DriveConstants.kPTurningPID, DriveConstants.kITurningPID, DriveConstants.kDTurningPID, DriveConstants.TurningPIDPeriod);
        turnPIDController.enableContinuousInput(DriveConstants.TurningPIDMinimum, DriveConstants.TurningPIDMaximum);
        drivePIDController = new PIDController(DriveConstants.kPDrivePID, DriveConstants.kIDrivePID, DriveConstants.kDDrivePID, DriveConstants.DrivePIDPeriod);

        resetEncoders();
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
    }

    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public double getDriveVelocityRPM(){
        return driveEncoder.getVelocity();
    }

    public double getDrivePositionMeters(){
        return driveEncoder.getPosition() * DriveConstants.RtoMeterConversion;
    }

    public double getDrivePositionRotations(){
        return driveEncoder.getPosition();
    }

    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePositionMeters(), Rotation2d.fromDegrees(getTurningPosition()));
    }

    public double getTurningPosition(){
        return canCoder.getAbsolutePosition().getValue() * 360 - 180 - canCoderOffset;//orginally + cancoder fofset
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocityRPM(), new Rotation2d(getTurningPosition() * Math.PI / 180));
    }

    public double getStateSpeedRPM(SwerveModuleState state){
        return state.speedMetersPerSecond * DriveConstants.MetersPerSecondtoRPMConversion;
    }

    public double getdrivePIDOutput(double velocitySetpoint){
        return drivePIDController.calculate(driveEncoder.getVelocity(), velocitySetpoint);
    }

    public double getturnPIDOutput(double turnSetpoint){
        return turnPIDController.calculate(getTurningPosition(), turnSetpoint);
    }

    public SwerveModuleState getCurrentState(){
        return new SwerveModuleState(0, Rotation2d.fromDegrees(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, getState().angle);

        turnMotor.set(-1 * getturnPIDOutput(state.angle.getDegrees()));
        //SmartDashboard.putNumber("Module " + canCoder.getDeviceID() + " RPM ", getStateSpeedRPM(state));
        driveMotor.set((driveMotorReversed) ? getdrivePIDOutput(getStateSpeedRPM(state)): -1 * getdrivePIDOutput(getStateSpeedRPM(state)));
        
        //SmartDashboard.putString("Module " + canCoder.getDeviceID() + " State ", state.toString());

        //SmartDashboard.putNumber("Module " + canCoder.getDeviceID() + " Turn Setpoint", turnPIDController.getSetpoint());
        //SmartDashboard.putNumber("Module " + canCoder.getDeviceID() + " Turn PV", getTurningPosition());
        //SmartDashboard.putNumber("Module " + canCoder.getDeviceID() + " Turn Error", turnPIDController.getPositionError());
        
        //SmartDashboard.putNumber("Module " + canCoder.getDeviceID() + " Drive Setpoint", drivePIDController.getSetpoint());
        //SmartDashboard.putNumber("Module " + canCoder.getDeviceID() + " Drive PV", driveEncoder.getVelocity());
    }
}
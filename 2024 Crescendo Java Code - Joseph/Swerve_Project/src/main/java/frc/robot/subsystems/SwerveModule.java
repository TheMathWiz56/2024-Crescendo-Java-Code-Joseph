package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;




public class SwerveModule {

    private final TalonFX driveMotor;

    private final TalonFX turnMotor;
    
    private final CANcoder canCoder;
    private final double canCoderOffset;

    private final PIDController turnPIDController;
    private final PIDController drivePIDController;

    public SwerveModule(int driveMotorid, int turnMotorid, boolean driveMotorReversed, boolean turnMotorReversed, int canCoderid, double canCoderOffset){

        driveMotor = new TalonFX(driveMotorid);
        turnMotor = new TalonFX(turnMotorid);

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
        driveMotor.setRotorPosition(0);
    }

    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public double getDriveVelocityRPM(){
        SmartDashboard.putNumber("Motor " + driveMotor.getDeviceID() + " Velocity", driveMotor.getVelocity().getValue() * 60);
        return driveMotor.getVelocity().getValue() * 60;
    }

    public double getDrivePositionMeters(){
        return driveMotor.getPosition().getValue() * DriveConstants.RtoMeterConversion;
    }

    public double getDrivePositionRotations(){
        SmartDashboard.putNumber("Motor " + driveMotor.getDeviceID() + " Position", driveMotor.getPosition().getValue());
        return driveMotor.getPosition().getValue();
    }

    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePositionMeters(), Rotation2d.fromDegrees(getTurningPosition()));
    }

    public double getTurningPosition(){
        return canCoder.getAbsolutePosition().getValue() * 360 - 180 - canCoderOffset;//removed  - cancoder offset
        
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocityRPM(), new Rotation2d(getTurningPosition() * Math.PI / 180));
    }

    public double getStateSpeedRPM(SwerveModuleState state){
        return state.speedMetersPerSecond * DriveConstants.MetersPerSecondtoRPMConversion;
    }

    public double getdrivePIDOutput(double velocitySetpoint){
        //SmartDashboard.putNumber("Motor " + driveMotor.getDeviceID() + " Velocity Setpoint", velocitySetpoint);
        if(velocitySetpoint == 0){
            return 0;
        }
        else{
            return drivePIDController.calculate(getDriveVelocityRPM(), velocitySetpoint);
        }
    }

    public double getturnPIDOutput(double turnSetpoint){
        return turnPIDController.calculate(getTurningPosition(), turnSetpoint);
    }

    public SwerveModuleState getCurrentState(){
        return new SwerveModuleState(0, Rotation2d.fromDegrees(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, getState().angle);

        turnMotor.set(-1 *getturnPIDOutput(state.angle.getDegrees()));
        driveMotor.set(getdrivePIDOutput(getStateSpeedRPM(state)));
    }
}
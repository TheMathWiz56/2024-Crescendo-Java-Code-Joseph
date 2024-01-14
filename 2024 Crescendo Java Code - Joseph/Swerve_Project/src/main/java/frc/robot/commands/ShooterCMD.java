package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

public class ShooterCMD extends CommandBase{
    
    private final Shooter shooter;
    private final Supplier <Boolean> forwards, backwards;
    
    public ShooterCMD (Shooter shooter, Supplier<Boolean> forwards, Supplier<Boolean> backwards){
        this.shooter = shooter;
        this.forwards = forwards;
        this.backwards = backwards;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (forwards.get()){
            shooter.setSpeed(1);
        }
        else if(backwards.get()){
            shooter.setSpeed(-1);
        }
        else{
            shooter.setSpeed(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

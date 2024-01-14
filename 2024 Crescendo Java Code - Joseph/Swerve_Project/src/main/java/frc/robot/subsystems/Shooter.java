package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Shooter extends SubsystemBase{

    private final CANSparkMax motor1, motor2;

    public Shooter(int motor1ID, int motor2ID){
        motor1 = new CANSparkMax(motor1ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor2 = new CANSparkMax(motor2ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    public void setSpeed(double speed){
        if (speed < 0){
            motor1.set(speed +.25);
            motor2.set(-speed);
        }
        else if (speed > 0){
            motor1.set(speed - .25);
            motor2.set(-speed);
        }
        else{
            motor1.set(speed);
            motor2.set(-speed);
        }
    }
    
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Intake extends SubsystemBase{

    private final CANSparkMax motor1;

    public Intake(int motor1ID){
        motor1 = new CANSparkMax(motor1ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    public void setSpeed(double speed){
        motor1.set(speed);
    }
    
}

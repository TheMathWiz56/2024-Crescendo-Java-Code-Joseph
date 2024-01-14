package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCMD extends CommandBase{

    private final Intake intake;
    private final Supplier <Boolean> forwards, backwards;
    
    public IntakeCMD (Intake intake, Supplier<Boolean> forwards, Supplier<Boolean> backwards){
        this.intake = intake;
        this.forwards = forwards;
        this.backwards = backwards;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (forwards.get()){
            intake.setSpeed(.5);
        }
        else if(backwards.get()){
            intake.setSpeed(-.5);
        }
        else{
            intake.setSpeed(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
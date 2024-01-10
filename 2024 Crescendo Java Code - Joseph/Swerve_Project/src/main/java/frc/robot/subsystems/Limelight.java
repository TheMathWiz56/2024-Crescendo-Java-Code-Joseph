package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers; //limelight lib
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

//Multiple pipelines are for multiple types of targetting / processing
//Should usually have one pipline per april tag
public class Limelight extends SubsystemBase{

    private final NetworkTable table;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    private final NetworkTableEntry tid;
    private int inttid = 0;
    private final NetworkTableEntry tbotpose;
    private boolean hasBotPose = false;
    private Pose2d botpose2d;

    public Limelight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tid = table.getEntry("tid");
        tbotpose = table.getEntry("botpose");
    }
    
    @Override
    public void periodic(){
        //read values periodically
        botpose2d = LimelightHelpers.getBotPose2d("limelight");

        //post to smart dashboard periodically
        SmartDashboard.putString("Limelight botpose2d", botpose2d.toString());
        inttid = (int)tid.getDouble(-1);
        if (inttid == -1){
            hasBotPose = false;
        }
        else{
            hasBotPose = true;
        }
    }

    public boolean getHasBotPose(){
        return hasBotPose;
    }

    public Pose2d getBotPose(){
        return botpose2d;
    }
}

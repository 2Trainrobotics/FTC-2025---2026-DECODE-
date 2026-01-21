package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;


    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

    }

    @Override
    public void start(){

    }

    @Override
    public void loop () {

    }
}

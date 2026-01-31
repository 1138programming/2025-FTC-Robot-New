package org.firstinspires.ftc.teamcode;

import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class Limelight {
    Limelight3A limelight;
    public Limelight(Limelight3A limelight){
        this.limelight = limelight;
        this.limelight.setPollRateHz(100);
        this.limelight.start();
        this.limelight.pipelineSwitch(0);
    }

    public double getTx(){
        LLResult result = limelight.getLatestResult();
        return result.getTx();
    }

    public double getTy(){
        LLResult result = limelight.getLatestResult();
        return result.getTy();
    }
    public int getTid(){
        LLResult result = limelight.getLatestResult();
        List<DetectorResult> detectorResults = result.getDetectorResults();

        if (!detectorResults.isEmpty()){
            return detectorResults.get(0).getClassId();
        }
        return 0;
    }

    public void setBotOrientation(double yaw){
        limelight.updateRobotOrientation(yaw);
    }

    public Pose3D getBotPose(){
        return limelight.getLatestResult().getBotpose_MT2();
    }


}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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


}

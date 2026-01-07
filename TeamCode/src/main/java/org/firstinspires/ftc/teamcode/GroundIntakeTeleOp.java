package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="GroundIntakeTeleOpBrighton4167")
public class GroundIntakeTeleOp extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, rightFront, leftBack, rightBack, flywheelMotor, intakeMotor;
    private IntegratingGyroscope gyro;
    private NavxMicroNavigationSensor navxMicro;

    @Override
    public void runOpMode(){

    }
}

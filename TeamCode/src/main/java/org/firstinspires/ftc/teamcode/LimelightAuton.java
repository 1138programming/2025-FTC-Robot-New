package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "LimelightAutonKatieIan4167", group = "Linear OpMode")
public class LimelightAuton extends LinearOpMode {
    private DcMotor leftFront, rightFront, leftBack, rightBack, intakeMotor;
    private Limelight limelight;
    private DcMotorEx flywheelMotor;
    private IntegratingGyroscope gyro;
    private NavxMicroNavigationSensor navxMicro;

    private final int encoderTicksPerRevolution = 28;
    private final float wheelCircumferenceIn = 3.75f;

    public void waitTime(int time){
        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < time){
            continue;
        }
    }


    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "Flywheel");
        Limelight3A limelight1 = hardwareMap.get(Limelight3A.class, "Limelight67");

        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope) navxMicro; //IntegratingGyroscope is a wrapper for navxmicronavigator
        limelight = new Limelight(limelight1);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        Drivebase drivebase = new Drivebase(leftFront, leftBack, rightFront, rightBack, gyro);
        waitForStart();
        float flyWheelVelocity = -0.75f;
        float intakeMotorVelocity = 0.8f;

        while(opModeIsActive()){
            telemetry.addData("dist", drivebase.getEncoderdist(leftFront));

            telemetry.addData("Tx", limelight.getTx());
            telemetry.update();
        }


/*
* rf-0
* lf-1
* lb-3
* rb-2
*
* indexer e - 1
* intake e - 3
*
* */

    }
}

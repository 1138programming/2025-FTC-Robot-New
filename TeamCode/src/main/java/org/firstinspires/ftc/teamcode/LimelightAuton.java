package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "LimelightAutonKatieIan4167", group = "Linear OpMode")
public class LimelightAuton extends LinearOpMode {
    private DcMotor leftFront, rightFront, leftBack, rightBack, intakeMotor;
    private Limelight3A limelight;
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
        //limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope) navxMicro; //IntegratingGyroscope is a wrapper for navxmicronavigator

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

        telemetry.addData("dist", drivebase.getEncoderdist(leftFront));
        telemetry.update();

        drivebase.driveDistance(5);
        // doesnt always end up in the same place due to drift. either overshotots or undershoots and the distance on screen does not align with actual distance


        telemetry.addData("dist", drivebase.getEncoderdist(leftFront));
        telemetry.update();

        waitTime(5000);

    }
}

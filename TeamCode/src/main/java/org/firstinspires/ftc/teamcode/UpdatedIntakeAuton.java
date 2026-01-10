package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="IntakeAutonBrighton4167", group="LinearOpMode")
public class UpdatedIntakeAuton extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, rightFront, leftBack, rightBack, flywheelMotor, intakeMotor;
    private CRServo indexer;
    private IntegratingGyroscope gyro;
    private NavxMicroNavigationSensor navxMicro;

    private Drivebase drivebase;
    private final float flyWheelVelocity = 0.767f;

    public void waitms(int ms){
        try {
            for (int i = 0; i < ms; i++) {
                if (opModeIsActive()) {
                    Thread.sleep(1);
                }
            }
        } catch (InterruptedException e){
            telemetry.addData("Something went horribly wrong", e);
            telemetry.update();
        }
    }

    public void runMotorsForTime(DcMotor intake, DcMotor flywheel, CRServo indexer, int ms, boolean reversed){ //worst method ever written
        boolean intakeFlag = intake != null;
        boolean flywheelFlag = flywheel != null;
        boolean indexerFlag = indexer != null;

        if (intakeFlag) intake.setPower(reversed ? -1 : 1);
        if (flywheelFlag) flywheel.setPower(reversed ? -flyWheelVelocity:flyWheelVelocity);
        if (indexerFlag) indexer.setPower(reversed ? -1 : 1);

        waitms(ms);
    }

    public void horizontalLeft(int time){
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(0.5);
        waitms(time);
    }

    public void runAuton(){
        drivebase.driveTime(0,-1, 0, false,  0.5f, 1800, this);
        flywheelMotor.setPower(flyWheelVelocity);
        waitms(4000);
        indexer.setPower(1);
        flywheelMotor.setPower(flyWheelVelocity);
        waitms(4000);
        flywheelMotor.setPower(flyWheelVelocity);
        waitms(4000);
        intakeMotor.setPower(-1);
        indexer.setPower(1);
        flywheelMotor.setPower(flyWheelVelocity);
        waitms(4000);
        flywheelMotor.setPower(flyWheelVelocity);
        waitms(4000);
        drivebase.driveTime(0, 0, 0.75f, false, 0.5f, 720, this);
        horizontalLeft(550);
        drivebase.driveTime(0, 1, 0, false, 0.5f, 400, this);
        intakeMotor.setPower(-1);

    }

    @Override
    public void runOpMode(){
        leftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        rightFront  = hardwareMap.get(DcMotor.class, "RightFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        flywheelMotor = hardwareMap.get(DcMotor.class, "Flywheel");
        indexer = hardwareMap.get(CRServo.class,"Indexer");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro; //integratinggyroscope is a wrapper for navxmicronavigator
        drivebase = new Drivebase(leftFront, leftBack, rightFront, rightBack, gyro);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        runAuton();




    }
}

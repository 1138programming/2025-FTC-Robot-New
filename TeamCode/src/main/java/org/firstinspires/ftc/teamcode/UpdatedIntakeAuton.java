package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="IntakeAutonBrighton4167", group="LinearOpMode")
public class UpdatedIntakeAuton extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, rightFront, leftBack, rightBack, flywheelMotor, intakeMotor;
    private CRServo Indexer;
    private IntegratingGyroscope gyro;
    private NavxMicroNavigationSensor navxMicro;

    private Drivebase drivebase;
    private final float flyWheelVelocity = 0.75f;

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

    public void runMotorsForTime(DcMotor intake, DcMotor flywheel, CRServo indexer, int ms, boolean reversed){
        boolean intakeFlag = intake != null;
        boolean flywheelFlag = flywheel != null;
        boolean indexerFlag = indexer != null;

        if (intakeFlag) intake.setPower(reversed ? -1 : 1);
        if (flywheelFlag) flywheel.setPower(reversed ? -flyWheelVelocity:flyWheelVelocity);
        if (indexerFlag) indexer.setPower(reversed ? -1 : 1);

        waitms(ms);
    }

    public void runAuton(){
        drivebase.driveTime(0,-1, 0, true,  0.5f, 1800, this);

    }

    @Override
    public void runOpMode(){
        leftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        rightFront  = hardwareMap.get(DcMotor.class, "RightFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        flywheelMotor = hardwareMap.get(DcMotor.class, "Flywheel");
        Indexer = hardwareMap.get(CRServo.class,"Indexer");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro; //integratinggyroscope is a wrapper for navxmicronavigator
        drivebase = new Drivebase(leftFront, leftBack, rightFront, rightBack, gyro);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();




            flywheelMotor.setPower(1);
            intakeMotor.setPower(1);
            waitms(2000);
            flywheelMotor.setPower(0);
            intakeMotor.setPower(0);




    }
}

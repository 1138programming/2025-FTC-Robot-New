package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="PreloadScoringAuton41", group="LinearOpMode")
public class PreLoadScoreAuton extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, rightFront, leftBack, rightBack, flywheelMotor, intakeMotor;
    private IntegratingGyroscope gyro;
    private NavxMicroNavigationSensor navxMicro;


    @Override
    public void runOpMode(){
        leftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        rightFront  = hardwareMap.get(DcMotor.class, "RightFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        flywheelMotor = hardwareMap.get(DcMotor.class, "Flywheel");

        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro; //integratinggyroscope is a wrapper for navxmicronavigator

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

        drivebase.driveTime(0,-1, 0, true,  0.5f, 1800, this);
        try {
            for (int i = 0; i < 4000; i++) { //rev flywheel before firing
                flywheelMotor.setPower(flyWheelVelocity);
//                if (i > 2000 && i < 3500){
//                    intakeMotor.setPower(1);
//                } else {
//                    intakeMotor.setPower(0);
//                }

                Thread.sleep(1);
            }
        } catch (InterruptedException e) {
            telemetry.addData("Something went horribly wrong", e);
        }
//first ball
        try {
            for (int i = 0; i < 500; i++) {
                intakeMotor.setPower(-1); //spin intake motor to move it into the shooter flywheel
                flywheelMotor.setPower(flyWheelVelocity);
                Thread.sleep(1);
            }
        } catch (InterruptedException e){
            telemetry.addData("Something went horribly wrong", e);
        }
//second ball
        try {
            for (int i = 0; i < 2000; i++) {
                flywheelMotor.setPower(flyWheelVelocity); //keep flywheel revved
                intakeMotor.setPower(1); //reset
                Thread.sleep(1);
            }
        } catch (InterruptedException e) {
            telemetry.addData("Something went horribly wrong", e);
        }

        try {
            for (int i = 0; i < 500; i++) {
                intakeMotor.setPower(-1); //move into flywheel
                flywheelMotor.setPower(flyWheelVelocity); //keep flywheel revved
                Thread.sleep(1);
            }
        } catch (InterruptedException e) {
            telemetry.addData("Something went horribly wrong", e);
        }
//third ball
        try {
            for (int i = 0; i < 2000; i++) {
                intakeMotor.setPower(1); //outtake
                flywheelMotor.setPower(flyWheelVelocity); //keep flywheel revved
                Thread.sleep(1);
            }
        } catch (InterruptedException e) {
            telemetry.addData("Something went horribly wrong", e);
        }

        try {
            for (int i = 0; i < 500; i++) {
                intakeMotor.setPower(-1); //move into flywheel
                flywheelMotor.setPower(flyWheelVelocity); //keep flywheel revved
                Thread.sleep(1);
            }
        } catch (InterruptedException e) {
            telemetry.addData("Something went horribly wrong", e);
        }












        while(opModeIsActive()){
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Auton", "67");
            telemetry.addData("Gyro1",  gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            telemetry.update();
        }






    }
}

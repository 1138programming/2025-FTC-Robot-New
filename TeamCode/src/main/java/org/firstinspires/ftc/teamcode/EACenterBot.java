package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name = "EACenterBotV1", group = "Linear OpMode")
public class EACenterBot extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Setup
        DcMotor releaseMotor = hardwareMap.get(DcMotor.class, "Release");
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "RightDrive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        releaseMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        boolean releaseMotorRun = false;
        boolean releaseMotorReset = false;
        final int encoderPos = 800;
        final int encoderResetPos = -400;
        //While Running
        while (opModeIsActive()) {
            telemetry.addData("releaseMotorEncoderPos:",releaseMotor.getCurrentPosition());
            telemetry.update();

            if (gamepad1.dpad_up) {
                releaseMotorRun = true;
            }

            if (releaseMotorReset && releaseMotor.getCurrentPosition() > encoderResetPos) {
                releaseMotor.setPower(-1.0);
            }
            else if (releaseMotorReset && releaseMotor.getCurrentPosition() <= encoderResetPos) {
                releaseMotor.setPower(0.0);
                releaseMotorRun = false;
                releaseMotorReset = false;
            }

            if (releaseMotorRun && releaseMotor.getCurrentPosition() < encoderPos) {
                releaseMotor.setPower(1.0);
            }
            else if (releaseMotorRun && releaseMotor.getCurrentPosition() >= encoderPos) {
                releaseMotor.setPower(0.0);
                releaseMotorRun = false;
                releaseMotorReset = true;
            }

            if (!releaseMotorReset && !releaseMotorRun) {
                if (releaseMotor.getCurrentPosition() > 100) {
                    releaseMotor.setPower(-1.0);
                }
                if (releaseMotor.getCurrentPosition() < -100) {
                    releaseMotor.setPower(1.0);
                }
                else {
                    releaseMotor.setPower(0.0);
                }
            }

            //Base Code Start
            float leftY = gamepad1.left_stick_y;
            float rightX = gamepad1.right_stick_x;

            leftDrive.setPower(leftY + rightX);
            rightDrive.setPower(leftY - rightX);

        }


    }


}

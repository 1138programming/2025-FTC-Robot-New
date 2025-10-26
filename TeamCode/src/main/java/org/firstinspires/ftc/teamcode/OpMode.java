/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp (name="OpMode 67", group="Linear OpMode")

public abstract class OpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left = null;
    private DcMotor right = null;
    private CRServo indexer = null;
    private IntegratingGyroscope gyro;
    private NavxMicroNavigationSensor navxMicro;
    private boolean start = false;
    private boolean reversed  = false;
    private boolean lastpress = false;


    public void runOpMode() {
            int stop = 2750;
            double speed = 5;
            boolean speedtoggle = false;
            int last = 0;
            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            DcMotor left  = hardwareMap.get(DcMotor.class, "Left_Front");
            DcMotor right  = hardwareMap.get(DcMotor.class, "Left_Back");
           // rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_Front");
            //rightBackDrive = hardwareMap.get(DcMotor.class, "Right_Back");

        //private DcMotorEx armMotor = null;
            DcMotor intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
           // teleMotor = hardwareMap.get(DcMotor.class, "tele");
            //armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

            //wristL = hardwareMap.get(CRServo.class, "wristL");
            //wristR = hardwareMap.get(CRServo.class, "wristR");
            indexer = hardwareMap.get(CRServo.class, "indexer");

            navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
            gyro = (IntegratingGyroscope)navxMicro;


            // ########################################################################################
            // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
            // ########################################################################################
            // Most robots need the motors on one side to be reversed to drive forward.
            // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
            // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
            // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
            // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
            // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
            // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
            left.setDirection(DcMotor.Direction.FORWARD);
            right.setDirection(DcMotor.Direction.FORWARD);
            //rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            //rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            indexer.setDirection(CRServo.Direction.FORWARD);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            // A timer helps provide feedback while calibration is taking place
            ElapsedTime timer = new ElapsedTime();

            // Wait for the game to start (driver presses START)
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();


            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                double left_right = gamepad1.left_stick_x;
                double up_down = gamepad1.right_stick_y;

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double left_motor_power = (left_right + up_down) * speed;
                double right_motor_power = (left_right - up_down) * speed;

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
               // max = Math.max(Math.abs(FrontPower), Math.abs(rightFrontPower));
              //  max = Math.max(max, Math.abs(rightBackPower));


                // This is test code:
                //
                // Uncomment the following code to test your motor directions.
                // Each button should make the corresponding motor run FORWARD.
                //   1) First get all the motors to take to correct positions on the robot
                //      by adjusting your Robot Configuration if necessary.
                //   2) Then make sure they run in the correct direction by modifying the
                //      the setDirection() calls above.
                // Once the correct motors move in the correct direction re-comment this code.

                /*
                FrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
                BackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
                rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
                rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
                */

                // Send calculated power to wheels
                left.setPower(left_motor_power);
                right.setPower(right_motor_power);

                if (gamepad1.y && !lastpress){
                    reversed = !reversed;
                    lastpress = true;
                }
                else if (!gamepad1.y){
                    lastpress = false;
                }

                // Turns on speedToggle when the left bumper is pressed
                // (starts fast, becomes slower when pressed)
                speedtoggle = gamepad1.left_bumper;


    // high 1850
    //low arm 1371 tele -1013
    // pickup 204


                /*
                if (gamepad2.right_bumper) {
                    start = true;
                    Wrist.setPosition(0.15);
                }
                else if (gamepad2.right_trigger > 0) {
                    start = true;
                    Wrist.setPosition(0.85);
                }
                else  if (start){
                    Wrist.setPosition(0.5);
                }
                else {
    //                Wrist.setPosition(0.85);
                     Wrist.setPosition(0.5);

                } */

               /* if(gamepad2.right_trigger > 0.1) {
                    claw.setPower(0.1);
                }
                else if (gamepad2.right_bumper) {
                    claw.setPower(0.9);
                }

                if(Math.abs(gamepad2.left_stick_y) > Math.abs(gamepad2.right_stick_x)) {
                    if (gamepad2.left_stick_y > 0.1) {
                        wristR.setPower(0.25);
                        wristL.setPower(0.25);
                    } else if (gamepad2.left_stick_y < -0.1) {
                        wristR.setPower(-0.25);
                        wristL.setPower(-0.25);
                    }
                    else {
                        wristR.setPower(0);
                        wristL.setPower(0);
                    }
                }
                else {
                    if (gamepad2.right_stick_x > 0.1) {
                        wristR.setPower(0.25);
                        wristL.setPower(-0.25);
                    } else if (gamepad2.right_stick_x < -0.1) {
                        wristR.setPower(-0.25);
                        wristL.setPower(0.25);
                    }
                    else {
                        wristR.setPower(0);
                        wristL.setPower(0);
                    }
                }
    //            if (gamepad2.a) {
    //                Roller.setPosition(0.1);
    //            }
    //            else if (gamepad2.b) {
    //                Roller.setPosition(0.9);
    //            }
    //            else {
    //                Roller.setPosition(0.5);
    //            }

                */

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                //telemetry.addData("Front left/Right", "%4.2f, %4.2f", FrontPower, rightFrontPower);
                //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", BackPower, rightBackPower);
                //telemetry.addData("Arm Pos raw",  armMotor.getCurrentPosition());
                //telemetry.addData("Arm Pos ",  getarmpos());            telemetry.addData("Arm Pid", armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
              //  telemetry.addData("Tele Pos",  teleMotor.getCurrentPosition());
               // telemetry.addData("Arm Vel", armMotor.getVelocity());
                //telemetry.addData("Arm target", armMotor.getTargetPosition());
                telemetry.addData("Last", last);
                // telemetry.addData("ArmisBusyt", armMotor.isBusy());
               // telemetry.addData("target",  armTarget);
    //            telemetry.addData("Gyro1",  gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
    //            telemetry.addData("Gyro2",  gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
                telemetry.addData("Angle",  gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
                telemetry.addData("Reversed:",reversed);
                telemetry.update();
            }
        }
    /*int armofset =0;
    public int getarmpos() {
        return armMotor.getCurrentPosition() - armofset;
        */

    }


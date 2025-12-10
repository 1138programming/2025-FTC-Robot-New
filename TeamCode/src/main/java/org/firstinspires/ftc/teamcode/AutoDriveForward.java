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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
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


@Autonomous(name="Autodrive67", group="Linear OpMode")

public class AutoDriveForward extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private IntegratingGyroscope gyro;
    private NavxMicroNavigationSensor navxMicro;
    private boolean reversed  = false;

    @Override
    public void runOpMode() {
            int last = 0;
            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            DcMotor leftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
            DcMotor rightFront  = hardwareMap.get(DcMotor.class, "RightFront");
            DcMotor leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
            DcMotor rightBack = hardwareMap.get(DcMotor.class, "RightBack");


            navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
            gyro = (IntegratingGyroscope)navxMicro; //integratinggyroscope is a wrapper for navxmicronavigator


            // ########################################################################################
            // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
            // ########################################################################################
            // Most robots need the motors ong one side to be reversed to drive forward.
            // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
            // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
            // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
            // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
            // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
            // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            //rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            //rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            //Indexer.setDirection(CRServo.Direction.FORWARD);
            //intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // A timer helps provide feedback while calibration is taking place
            ElapsedTime timer = new ElapsedTime();

            // Wait for the game to start (driver presses START)
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            Drivebase drivebase = new Drivebase(leftFront, leftBack, rightFront, rightBack, gyro);

            drivebase.drive(0,1,0,false,0.1f);

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {


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
                telemetry.addData("Gyro1",  gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
//                telemetry.addData("Gyro2",  gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
                telemetry.addData("Reversed:",reversed);
                telemetry.update();
            }
        }
}


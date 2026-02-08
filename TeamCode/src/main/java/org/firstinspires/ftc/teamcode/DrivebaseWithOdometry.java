package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DrivebaseWithOdometry extends Drivebase{


    Orientation orientation;
    OpenGLMatrix RotMatrix;
    AngularVelocity angle;
    double drivekP, drivekI, drivekD, drivekF;
    double rotationkP, rotationkI, rotationkD, rotationkF;
    GoBildaPinpointDriver xOdoWheel;
    GoBildaPinpointDriver yOdoWheel;
    public DrivebaseWithOdometry(DcMotor LF, DcMotor LB, DcMotor RF, DcMotor RB, IntegratingGyroscope gyro, GoBildaPinpointDriver xOdoWheel, GoBildaPinpointDriver yOdoWheel){
        super(LF, LB, RF, RB, gyro);
        orientation = gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        RotMatrix =  orientation.getRotationMatrix();

        RotMatrix.get(0,0);
        angle = gyro.getAngularVelocity(AngleUnit.DEGREES);

        this.xOdoWheel = xOdoWheel;
        this.yOdoWheel = yOdoWheel;
        orientation.getRotationMatrix().getData();
        drivekP = 0.4;
        drivekI = 0.000001;
        drivekD = 0.002;
        drivekF = 0;

        rotationkP = 0.02;
        rotationkI = 0;
        rotationkD = 0.01;
        rotationkF = 0;
    }






}

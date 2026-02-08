package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Objects;

public class Flywheel {
    DcMotorEx flywheelMotor;
    PIDFController flywheelPIDController;
    Telemetry telemetry;
    public final double kP = 0.02;
    public final double kI = 0.000001;
    public final double kD = 0;
    public final double kF = 0;
    public final float encoderTicksPerRotation =  28f; //537.7f



    public Flywheel(DcMotorEx flywheelMotor){
        this.flywheelMotor = flywheelMotor;
        this.flywheelPIDController = new PIDFController(kP, kI, kD, kF);
    }

    public void addTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void spinAtRPM(int RPM, int time) {
        flywheelPIDController.setSetPoint(RPM);
        flywheelPIDController.setTolerance(20);

        int curRPM;
        double motorOutputRaw;
        double motorOutput;
        double flywheelVelocity;

        long startTime = System.currentTimeMillis();
        long curTime = System.currentTimeMillis() + 20;

        while (!flywheelPIDController.atSetPoint() || curTime - startTime < time) {
                calculateFlywheelPID();
        }
    }

        public void assignPIDTarget(int RPM){
            flywheelPIDController.setSetPoint(RPM);
        }

        public void calculateFlywheelPID() { //Assumes pid target and tolerance was set!!!
            if (flywheelPIDController.getSetPoint() == 0.0){
                throw new Exception("Flywheel PID Target not set!!");
            }
            flywheelPIDController.setTolerance(20);
            double flywheelVelocity = flywheelMotor.getVelocity();
            int curRPM = (int) (flywheelVelocity / encoderTicksPerRotation);
            curRPM *= 60;
            double motorOutputRaw = flywheelPIDController.calculate(curRPM);
            double motorOutput = Math.abs(motorOutputRaw) > 1 ? 1 : motorOutputRaw;

            flywheelMotor.setPower(motorOutput);
            telemetry.addData("Motor Velocity", flywheelVelocity);
            telemetry.addData("RPM", curRPM);
            telemetry.addData("Motor Output", motorOutput);
            telemetry.update();

        }
    }






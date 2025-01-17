package org.firstinspires.ftc.teamcode.shared;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


public class LiftPIDF {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static double target = 0;
    private static double offset = -30 + 360;
    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;
    private AnalogInput encoder;
    // private final double ticksInDegree = 140 / 360.0;

    public LiftPIDF(DcMotorEx motorOne, DcMotorEx motorTwo, AnalogInput encoder) {
        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
        this.encoder = encoder;
    }

    public void setTarget(double t) {
        target = t;
    }

    public void loop() {
        double armPos = (encoder.getVoltage() / 3.2 * 360 + offset) % 360;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(armPos)) * f;
        double power = pid + ff;
        motorOne.setPower(power);
        motorTwo.setPower(power);
    }

    public class LiftUp implements Action {
        ElapsedTime timer;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            if (timer == null) {
                timer = new ElapsedTime();
            }
            setTarget(60);
            return timer.milliseconds() < 500;
        }
    }
    public Action LiftUp() {
        return new LiftUp();
    }

    public class LiftDown implements Action {
        ElapsedTime timer;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            if (timer == null) {
                timer = new ElapsedTime();
            }
            setTarget(offset);
            return timer.milliseconds() < 500;
        }
    }
    public Action LiftDown() {
        return new LiftDown();
    }
}


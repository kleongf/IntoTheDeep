package org.firstinspires.ftc.teamcode.shared;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// the reason why i believe a PIDF is necessary is because an extended arm causes a lot more torque
// than a non-extended one, so more power must be sent to the motor
public class LiftPIDF {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static int target = 0;
    private DcMotorEx liftMotor;

    public LiftPIDF(DcMotorEx motor) {
        controller = new PIDController(p, i, d);
        liftMotor = motor;
    }

    public void setTarget(int t) {
        target = t;
    }

    public void loop() {
        controller.setPID(p, i, d);
        int armPos = liftMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double power = pid;

        liftMotor.setPower(power);
    }

    public class LiftUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            setTarget(500);
            return false;
        }
    }
    public Action LiftUp() {
        return new LiftUp();
    }

    public class LiftFirst implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            setTarget(1000);
            return false;
        }
    }
    public Action LiftFirst() {
        return new LiftFirst();
    }

    public class LiftSecond implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            setTarget(2000);
            return false;
        }
    }
    public Action LiftSecond() {
        return new LiftSecond();
    }

    public class LiftFirstDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            setTarget(1000);
            return false;
        }
    }
    public Action LiftFirstDown() {
        return new LiftFirstDown();
    }

    public class LiftDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            setTarget(0);
            return false;
        }
    }
    public Action LiftDown() {
        return new LiftDown();
    }
}

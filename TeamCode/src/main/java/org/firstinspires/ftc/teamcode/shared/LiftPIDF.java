package org.firstinspires.ftc.teamcode.shared;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

// the reason why i believe a PIDF is necessary is because an extended arm causes a lot more torque
// than a non-extended one, so more power must be sent to the motor
public class LiftPIDF {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;
    private DcMotorEx liftMotor;
    private double ticksInDegree = (1000.0 / 180.0);

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
        double ff = Math.sin(Math.toRadians(target / ticksInDegree)) * f;
        double power = pid + ff;

        liftMotor.setPower(power);
    }

    public class LiftUp implements Action {
        ElapsedTime timer;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // will still need some tuning
            if (timer == null) {
                timer = new ElapsedTime();
            }
            setTarget(500);
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
            setTarget(0);
            return timer.milliseconds() < 500;
        }
    }
    public Action LiftDown() {
        return new LiftDown();
    }
}

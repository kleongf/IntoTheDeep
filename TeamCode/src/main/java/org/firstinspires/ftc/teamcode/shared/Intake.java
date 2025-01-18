package org.firstinspires.ftc.teamcode.shared;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private Servo motorOne;
    private Servo motorTwo;
    private CRServo intakeMotor;

    public Intake(Servo motorOne, Servo motorTwo, CRServo intakeMotor) {
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
        this.intakeMotor = intakeMotor;
    }

    public void IntakeForward() {
        intakeMotor.setPower(0.5);
    }

    public void IntakeReverse() {
        intakeMotor.setPower(-0.5);
    }

    public void IntakeStop() {
        intakeMotor.setPower(0);
    }

    public void IntakeUp() {
        motorOne.setPosition(1);
    }

    public void IntakeDown() {
        motorOne.setPosition(-1);
    }

    public void IntakeStraight() {
        motorOne.setPosition(0);
    }

    public class IntakeForwardAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            IntakeForward();
            return false;
        }
    }
    public Action IntakeForwardAction() {
        return new IntakeForwardAction();
    }

    public class IntakeReverseAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            IntakeReverse();
            return false;
        }
    }
    public Action IntakeReverseAction() {
        return new IntakeReverseAction();
    }

    public class IntakeStopAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            IntakeStop();
            return false;
        }
    }
    public Action IntakeStopAction() {
        return new IntakeStopAction();
    }

    public class IntakeUpAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            IntakeUp();
            return false;
        }
    }
    public Action IntakeUpAction() {
        return new IntakeUpAction();
    }

    public class IntakeDownAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            IntakeDown();
            return false;
        }
    }
    public Action IntakeDownAction() {
        return new IntakeDownAction();
    }

    public class IntakeStraightAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            IntakeStraight();
            return false;
        }
    }
    public Action IntakeStraightAction() {
        return new IntakeStraightAction();
    }
}

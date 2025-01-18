package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shared.ExtendPIDF;
import org.firstinspires.ftc.teamcode.shared.Intake;
import org.firstinspires.ftc.teamcode.shared.LiftPIDF;

@TeleOp(name="servo testing", group="Linear OpMode")
public class ServoTesting extends OpMode {

    public Servo rotateMotorTwo;
    public Servo rotateMotorOne;
    public CRServo intakeMotor;


    @Override
    public void init() {
        rotateMotorOne = hardwareMap.get(Servo.class, "rotateMotorOne");
        rotateMotorTwo = hardwareMap.get(Servo.class, "rotateMotorTwo");
        intakeMotor = hardwareMap.get(CRServo.class, "intakeMotor");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            intakeMotor.setPower(-0.5);
        }
        if (gamepad1.b) {
            intakeMotor.setPower(0.5);
        }
        if (gamepad1.left_bumper) {
            rotateMotorTwo.setPosition(0.5);
        }
        if (gamepad1.right_bumper) {
            rotateMotorTwo.setPosition(-0.5);
        }

    }

}



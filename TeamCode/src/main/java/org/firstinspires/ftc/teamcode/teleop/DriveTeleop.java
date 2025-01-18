package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shared.ExtendPIDF;
import org.firstinspires.ftc.teamcode.shared.LiftPIDF;


@TeleOp(name="Drive TeleOp", group="Linear OpMode")
public class DriveTeleop extends OpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "left_front");
        frontRight = hardwareMap.get(DcMotorEx.class, "right_front");
        backLeft = hardwareMap.get(DcMotorEx.class, "left_back");
        backRight = hardwareMap.get(DcMotorEx.class, "right_back");
    }

    @Override
    public void loop() {
        double y = -this.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = this.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = this.gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        // some of the motors are reversed
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(-backLeftPower); // corrected
        frontRight.setPower(-frontRightPower);
        backRight.setPower(backRightPower);
    }
}

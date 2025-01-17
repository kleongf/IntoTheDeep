package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class ExtendPIDFTuning extends OpMode {
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.005;
    public static double f = 0;
    public static int target = 0;
    // private static double offset = -15 + 360;
    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;
    private AnalogInput encoder;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
        // motorOne = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        motorTwo = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        double armPos = motorTwo.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        // force of gravity on arm: at 0 degrees, it should be 0 and at 90 degrees, it should be max
        double ff = Math.sin(Math.toRadians(encoder.getVoltage() / 3.2 * 360)) * f;
        double power = pid + ff;
        motorTwo.setPower(power);
        // motorOne.setTargetPosition(target);
        // motorTwo.setPower(-power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();

    }
}


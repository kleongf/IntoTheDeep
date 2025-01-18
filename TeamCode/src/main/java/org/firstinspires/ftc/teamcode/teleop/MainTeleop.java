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


@TeleOp(name="Main TeleOp", group="Linear OpMode")
public class MainTeleop extends OpMode {

    public enum LiftOneState {
        LIFT_START,
        LIFT_UP,
        LIFT_EXTEND,
        LIFT_RETRACT,
        LIFT_DOWN
    };

    public enum LiftTwoState {
        LIFT_START,
        LIFT_UP,
        LIFT_EXTEND,
        LIFT_RETRACT,
        LIFT_DOWN
    };

    // the servo has setPosition from 0 to 1 (0 to 180 degrees)
    // therefore 90 degrees is setPosition(0.5)

    LiftOneState liftOneState = LiftOneState.LIFT_START;
    LiftTwoState liftTwoState = LiftTwoState.LIFT_START;

    LiftPIDF liftPIDF;
    ExtendPIDF extendPIDF;
    ElapsedTime intakeTimer;
    Intake intake;

    // lift high: high basket
    public int LIFT_HIGH = 80;
    // lift low: low basket
    public int LIFT_LOW = 50;
    // lift zero: ground level
    public int LIFT_ZERO = 0;
    // lift high:
    public int EXTEND_HIGH = 100;
    public int EXTEND_LOW = 50;
    public int EXTEND_ZERO = 0;

    public DcMotorEx liftMotorOne;
    public DcMotorEx liftMotorTwo;
    public DcMotorEx extendMotorOne;
    public DcMotorEx extendMotorTwo;
    public Servo rotateMotorOne;
    public Servo rotateMotorTwo;
    public CRServo intakeMotor;
    private AnalogInput analogEncoder;

    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    // private Servo servoMotor;

    @Override
    public void init() {
        liftMotorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        analogEncoder = hardwareMap.get(AnalogInput.class, "encoder");
        extendMotorOne = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        extendMotorTwo = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");
        rotateMotorOne = hardwareMap.get(Servo.class, "rotateMotorOne");
        rotateMotorTwo = hardwareMap.get(Servo.class, "rotateMotorTwo");
        intakeMotor = hardwareMap.get(CRServo.class, "intakeMotor");
        frontLeft = hardwareMap.get(DcMotorEx.class, "left_front");
        frontRight = hardwareMap.get(DcMotorEx.class, "right_front");
        backLeft = hardwareMap.get(DcMotorEx.class, "left_back");
        backRight = hardwareMap.get(DcMotorEx.class, "right_back");


        liftPIDF = new LiftPIDF(liftMotorOne, liftMotorTwo, analogEncoder);
        extendPIDF = new ExtendPIDF(extendMotorOne, extendMotorTwo);
        intake = new Intake(rotateMotorOne, rotateMotorTwo, intakeMotor);
        intakeTimer = new ElapsedTime();
    }

    @Override
    public void loop() {
        switch (liftOneState) {
            case LIFT_START:
                // Waiting for some input
                if (gamepad1.left_bumper) {
                    liftPIDF.setTarget(LIFT_HIGH);
                    liftOneState = LiftOneState.LIFT_UP;
                }
                break;
            case LIFT_UP:
                // check if the lift has finished extending,
                // otherwise do nothing.
                extendPIDF.setTarget(EXTEND_HIGH);
                if (Math.abs(analogEncoder.getVoltage() * 3.2 / 360 - LIFT_HIGH) < 5) {
                    // our threshold is within 5 degrees of target
                    liftOneState = LiftOneState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                if (gamepad1.right_trigger > 0.5) {
                    intake.IntakeReverse();
                    // make sure intake is on for enough time
                    if (intakeTimer.milliseconds() > 200) {
                        intakeTimer.reset();
                        liftOneState = LiftOneState.LIFT_RETRACT;
                    }
                }
                break;
            case LIFT_RETRACT:
                extendPIDF.setTarget(EXTEND_ZERO);
                if (Math.abs(extendMotorTwo.getCurrentPosition() - EXTEND_ZERO) < 10) {
                    liftOneState = LiftOneState.LIFT_DOWN;
                }
                break;
            case LIFT_DOWN:
                liftPIDF.setTarget(LIFT_ZERO);
                if (Math.abs(analogEncoder.getVoltage() * 3.2 / 360 - LIFT_ZERO) < 5) {
                    liftOneState = LiftOneState.LIFT_START;
                }
                break;
            default:
                // should never be reached, as liftState should never be null
                liftOneState = LiftOneState.LIFT_START;
        }

        switch (liftTwoState) {
            case LIFT_START:
                // Waiting for some input
                if (gamepad1.left_trigger > 0.5) {
                    liftPIDF.setTarget(LIFT_LOW);
                    liftTwoState = LiftTwoState.LIFT_UP;
                }
                break;
            case LIFT_UP:
                // check if the lift has finished extending,
                // otherwise do nothing.
                extendPIDF.setTarget(EXTEND_LOW);
                if (Math.abs(analogEncoder.getVoltage() * 3.2 / 360 - LIFT_LOW) < 5) {
                    // our threshold is within 10 encoder ticks of our target
                    liftTwoState = LiftTwoState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                if (gamepad1.left_trigger > 0.5) {
                    intake.IntakeReverse();
                    // make sure intake is on for enough time
                    if (intakeTimer.milliseconds() > 200) {
                        intakeTimer.reset();
                        liftTwoState = LiftTwoState.LIFT_RETRACT;
                    }
                }
                break;
            case LIFT_RETRACT:
                extendPIDF.setTarget(EXTEND_ZERO);
                if (Math.abs(extendMotorTwo.getCurrentPosition() - EXTEND_ZERO) < 10) {
                    liftTwoState = LiftTwoState.LIFT_DOWN;
                }
                break;
            case LIFT_DOWN:
                liftPIDF.setTarget(LIFT_ZERO);
                if (Math.abs(analogEncoder.getVoltage() * 3.2 / 360 - LIFT_HIGH) < 5) {
                    liftTwoState = LiftTwoState.LIFT_START;
                }
                break;
            default:
                // should never be reached, as liftState should never be null
                liftTwoState = LiftTwoState.LIFT_START;
        }

        if (gamepad1.right_bumper) {
            intake.IntakeForward();
            if (intakeTimer.milliseconds() > 200) {
                intakeTimer.reset();
            }
        }

        if (gamepad1.a) {
            // ascent here
        }

        // call motor set power here for PID
        liftPIDF.loop();
        extendPIDF.loop();
        // only thing that the finite state machine is doing is setting the target postiion
        double y = -this.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = this.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = this.gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        // some of the motors are reversed
//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(-backLeftPower);
        frontRight.setPower(-frontRightPower);
        backRight.setPower(backRightPower);
    }

}

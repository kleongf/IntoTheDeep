package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.shared.ExtendPIDF;
import org.firstinspires.ftc.teamcode.shared.Intake;
import org.firstinspires.ftc.teamcode.shared.LiftPIDF;


@Autonomous(name="Autonomous Red to Bucket", group="Autonomous")
public class AutoRedBucket extends LinearOpMode {

    @Override
    public void runOpMode() {
        // new understanding is that the robot's arm has a large range of motion (therefore it can move all the way back to grab a block)
        // idk how useful the extension part is rn so i will not put it in now

        Pose2d initialPose = new Pose2d(-36, -60, Math.toRadians(90));
        Pose2d dropOffPose = new Pose2d(-54, -54, Math.toRadians(45));
        Vector2d dropOff = new Vector2d(-54, -54);

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        DcMotorEx liftMotorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        DcMotorEx liftMotorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");
        DcMotorEx extendMotorOne = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        DcMotorEx extendMotorTwo = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");
        Servo rotateMotorOne = hardwareMap.get(Servo.class, "rotateMotorOne");
        Servo rotateMotorTwo = hardwareMap.get(Servo.class, "rotateMotorTwo");
        CRServo intakeMotor = hardwareMap.get(CRServo.class, "intakeMotor");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "encoder");

        ExtendPIDF extend = new ExtendPIDF(extendMotorOne, extendMotorTwo);
        LiftPIDF lift = new LiftPIDF(liftMotorOne, liftMotorTwo, encoder);
        Intake intake = new Intake(rotateMotorOne, rotateMotorTwo, intakeMotor);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .splineTo(dropOff, Math.toRadians(45));

        TrajectoryActionBuilder traj2 = drive.actionBuilder(dropOffPose)
                .splineTo(new Vector2d(-36, -24), Math.toRadians(180));

        TrajectoryActionBuilder traj3 = drive.actionBuilder(new Pose2d(-36, -24, Math.toRadians(180)))
                .splineTo(dropOff, Math.toRadians(235));

        TrajectoryActionBuilder traj4 = drive.actionBuilder(dropOffPose)
                .splineTo(new Vector2d(-46, -24), Math.toRadians(180));

        TrajectoryActionBuilder traj5 = drive.actionBuilder(new Pose2d(-46, -24, Math.toRadians(180)))
                .splineTo(dropOff, Math.toRadians(235));

        TrajectoryActionBuilder traj6 = drive.actionBuilder(dropOffPose)
                .splineTo(new Vector2d(-56, -24), Math.toRadians(180));

        TrajectoryActionBuilder traj7 = drive.actionBuilder(new Pose2d(-56, -24, Math.toRadians(180)))
                .splineTo(dropOff, Math.toRadians(235));

        TrajectoryActionBuilder traj8 = drive.actionBuilder(dropOffPose)
                .turn(Math.toRadians(125))
                .lineToX(45);

//                .splineTo(dropOff, Math.toRadians(45))
//                .splineTo(new Vector2d(-48, -40), Math.toRadians(90))
//                .strafeTo(dropOff)
//                .turn(Math.toRadians(-45))
//                .splineTo(new Vector2d(-58, -40), Math.toRadians(90))
//                .strafeTo(dropOff)
//                .turn(Math.toRadians(-45))
//                .splineTo(new Vector2d(-68, -40), Math.toRadians(90))
//                .strafeTo(dropOff)
//                .turn(Math.toRadians(-45))
//                .splineTo(new Vector2d(-16, -10), Math.toRadians(0))
//                .strafeTo(dropOff)
//                .turn(Math.toRadians(45))
//                .splineTo(new Vector2d(-16, -10), Math.toRadians(0))
//                .strafeTo(dropOff)
//                .turn(Math.toRadians(45));
        class LoopPID {
            class RunPID implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    lift.loop();
                    extend.loop();
                    return false;
                }
            }

            public Action runPID() {
                return new RunPID();
            }
        }

        LoopPID looper = new LoopPID();


        // intriguing i will look at this later
        // Actions.runBlocking(new MecanumDrive.FollowTrajectoryAction());

        waitForStart();

        Actions.runBlocking(
                // make it a parallel action that loops the pid
                new ParallelAction(
                        looper.runPID(),
                        new SequentialAction(
                                new ParallelAction(
                                        traj1.build(),
                                        lift.LiftUp(),
                                        extend.ExtendFirst()
                                ),
                                intake.IntakeReverseAction(),
                                new ParallelAction(
                                        traj2.build(),
                                        lift.LiftDown(),
                                        extend.Retract()
                                ),
                                intake.IntakeForwardAction(),
                                new ParallelAction(
                                        traj3.build(),
                                        lift.LiftUp(),
                                        extend.ExtendFirst()
                                ),
                                intake.IntakeReverseAction(),
                                new ParallelAction(
                                        traj4.build(),
                                        lift.LiftDown(),
                                        extend.Retract()
                                ),
                                intake.IntakeForwardAction(),
                                new ParallelAction(
                                        traj5.build(),
                                        lift.LiftUp(),
                                        extend.ExtendFirst()
                                ),
                                intake.IntakeReverseAction(),
                                new ParallelAction(
                                        traj6.build(),
                                        lift.LiftDown(),
                                        extend.Retract()
                                ),
                                intake.IntakeForwardAction(),
                                new ParallelAction(
                                        traj7.build(),
                                        lift.LiftUp(),
                                        extend.ExtendFirst()
                                ),
                                intake.IntakeReverseAction()
                                // traj8.build()
                        )
                )
        );

    }
}



package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.shared.ExtendPIDF;
import org.firstinspires.ftc.teamcode.shared.Intake;
import org.firstinspires.ftc.teamcode.shared.LiftPIDF;

@Autonomous(name="Autonomous to Submersible", group="Autonomous")
public class AutoSubmersible extends LinearOpMode {

    @Override
    public void runOpMode() {
        // new understanding is that the robot's arm has a large range of motion (therefore it can move all the way back to grab a block)
        // idk how useful the extension part is rn so i will not put it in now

        Pose2d initialPose = new Pose2d(-16, 60, Math.toRadians(90));
        Pose2d dropOffPose = new Pose2d(-50, 60, Math.toRadians(90));
        Pose2d pickUpPose = new Pose2d(30, 66, Math.toRadians(90));
        Vector2d dropOff = new Vector2d(-50, 60);
        Vector2d pickUp = new Vector2d(30, 66);

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        DcMotorEx extendMotor = hardwareMap.get(DcMotorEx.class, "extendMotor");
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        ExtendPIDF extend = new ExtendPIDF(extendMotor);
        LiftPIDF lift = new LiftPIDF(liftMotor);
        Intake intake = new Intake(intakeMotor);

        // this is part 1: dropping off all the blocks

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .lineToY(24);

        TrajectoryActionBuilder traj2 = drive.actionBuilder(new Pose2d(24, 30, Math.toRadians(90)))
                .strafeTo(new Vector2d(-50, 32));

        TrajectoryActionBuilder traj3 = drive.actionBuilder(new Pose2d(-50, 32, Math.toRadians(90)))
                .strafeTo(dropOff);

        TrajectoryActionBuilder traj4 = drive.actionBuilder(dropOffPose)
                .strafeTo(new Vector2d(-58, 32));

        TrajectoryActionBuilder traj5 = drive.actionBuilder(new Pose2d(-58, 32, Math.toRadians(90)))
                .strafeTo(dropOff);

        TrajectoryActionBuilder traj6 = drive.actionBuilder(dropOffPose)
                .strafeTo(new Vector2d(-66, 32));

        TrajectoryActionBuilder traj7 = drive.actionBuilder(new Pose2d(-66, 32, Math.toRadians(90)))
                .strafeTo(dropOff);


        // part 2: hanging all the blocks

        TrajectoryActionBuilder traj8 = drive.actionBuilder(dropOffPose)
                .strafeTo(new Vector2d(-12, 24));

        TrajectoryActionBuilder traj9 = drive.actionBuilder(new Pose2d(-12, 24, Math.toRadians(90)))
                .strafeTo(dropOff);

        TrajectoryActionBuilder traj10 = drive.actionBuilder(dropOffPose)
                .strafeTo(new Vector2d(-8, 24));

        TrajectoryActionBuilder traj11 = drive.actionBuilder(new Pose2d(-8, 24, Math.toRadians(90)))
                .strafeTo(dropOff);

        TrajectoryActionBuilder traj12 = drive.actionBuilder(dropOffPose)
                .strafeTo(new Vector2d(-4, 24));

        TrajectoryActionBuilder traj13 = drive.actionBuilder(new Pose2d(-4, 24, Math.toRadians(90)))
                .strafeTo(dropOff);

        TrajectoryActionBuilder traj14 = drive.actionBuilder(dropOffPose)
                .strafeTo(new Vector2d(0, 24));

        if (isStopRequested()) return;
        waitForStart();

        // hopefully this works and supplies power to the motor?
        while (opModeIsActive()) {
            lift.loop();
            // havent used extend but could be useful when hanging specimens
            extend.loop();
            // intake.loop() is not necessary bc not a pid
        }

        // part 1

        Actions.runBlocking(
                new SequentialAction(
                    traj1.build(),
                    new ParallelAction(
                            lift.LiftDown(),
                            intake.IntakeReverse() // implement a smart timer if needed
                    ),
                    traj2.build(),
                    intake.IntakeForward(),
                    traj3.build(),
                    new SequentialAction(
                            lift.LiftUp(), // should rotate arm backward
                            intake.IntakeReverse(),
                            lift.LiftDown()
                    ),
                    traj4.build(),
                    intake.IntakeForward(), // again, could be for a fixed amt of time
                    traj5.build(),
                        new SequentialAction(
                                lift.LiftUp(), // should rotate arm backward
                                intake.IntakeReverse(),
                                lift.LiftDown()
                        ),
                    traj6.build(),
                    intake.IntakeForward(),
                    traj7.build(),
                    intake.IntakeReverse()
                )
        );

        // part 2 (didn't put all of them for now)
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                lift.LiftUp(),
                                traj8.build()
                        ),
                        intake.IntakeForward(),
                        intake.IntakeReverse(),
                        traj9.build(),
                        lift.LiftDown(),
                        new ParallelAction(
                                lift.LiftUp(),
                                traj10.build()
                        ),
                        intake.IntakeForward(),
                        intake.IntakeReverse(),
                        traj11.build(),
                        lift.LiftDown()
                )
        );

    }
}

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

@Autonomous(name="Autonomous Red to Submersible", group="Autonomous")
public class AutoRedSubmersible extends LinearOpMode {

    @Override
    public void runOpMode() {
        // new understanding is that the robot's arm has a large range of motion (therefore it can move all the way back to grab a block)
        // idk how useful the extension part is rn so i will not put it in now

        Pose2d initialPose = new Pose2d(16, -60, Math.toRadians(90));
        Pose2d dropOffPose = new Pose2d(50, -60, Math.toRadians(90));
        Vector2d dropOff = new Vector2d(50, -60);

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

//        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
//        DcMotorEx extendMotor = hardwareMap.get(DcMotorEx.class, "extendMotor");
//        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//
//        ExtendPIDF extend = new ExtendPIDF(extendMotor);
//        LiftPIDF lift = new LiftPIDF(liftMotor);
//        Intake intake = new Intake(intakeMotor);


        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .lineToY(-24)
                .strafeTo(new Vector2d(50, -32))
                .strafeTo(dropOff)
                .strafeTo(new Vector2d(58, -32))
                .strafeTo(dropOff)
                .strafeTo(new Vector2d(66, -32))
                .strafeTo(dropOff);

        TrajectoryActionBuilder traj2 = drive.actionBuilder(dropOffPose)
                .strafeTo(new Vector2d(12, -24))
                .strafeTo(dropOff)
                .strafeTo(new Vector2d(9, -24))
                .strafeTo(dropOff)
                .strafeTo(new Vector2d(6, -24))
                .strafeTo(dropOff)
                .strafeTo(new Vector2d(3, -24));

        if (isStopRequested()) return;
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        traj1.build(),
                        traj2.build()
                )
        );

    }
}



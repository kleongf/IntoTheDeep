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

@Autonomous(name="Autonomous Red to bucket", group="Autonomous")
public class AutoRedBucket extends LinearOpMode {

    @Override
    public void runOpMode() {
        // new understanding is that the robot's arm has a large range of motion (therefore it can move all the way back to grab a block)
        // idk how useful the extension part is rn so i will not put it in now

        Pose2d initialPose = new Pose2d(-48, -60, Math.toRadians(90));
        Pose2d dropOffPose = new Pose2d(-54, -54, Math.toRadians(90));
        Vector2d dropOff = new Vector2d(-54, -54);

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

//        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
//        DcMotorEx extendMotor = hardwareMap.get(DcMotorEx.class, "extendMotor");
//        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//
//        ExtendPIDF extend = new ExtendPIDF(extendMotor);
//        LiftPIDF lift = new LiftPIDF(liftMotor);
//        Intake intake = new Intake(intakeMotor);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .splineTo(dropOff, Math.toRadians(45))
                .splineTo(new Vector2d(-54, -40), Math.toRadians(90))
                .strafeTo(dropOff)
                .turn(Math.toRadians(-45))
                .splineTo(new Vector2d(-60, -40), Math.toRadians(90))
                .strafeTo(dropOff)
                .turn(Math.toRadians(-45))
                .splineTo(new Vector2d(-66, -40), Math.toRadians(90))
                .strafeTo(dropOff)
                .turn(Math.toRadians(-45))
                .splineTo(new Vector2d(-16, -10), Math.toRadians(0))
                .strafeTo(dropOff)
                .turn(Math.toRadians(45))
                .splineTo(new Vector2d(-16, -10), Math.toRadians(0))
                .strafeTo(dropOff)
                .turn(Math.toRadians(45));


        Actions.runBlocking(
                new SequentialAction(
                        traj1.build()
                )
        );

    }
}


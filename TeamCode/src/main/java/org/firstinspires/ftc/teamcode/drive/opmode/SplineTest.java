package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(-40,-63));
        //drive.setPoseEstimate(new Pose2d(90, 63));
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-40,-63))
                .splineTo(new Vector2d(-28,-38),0)
                .splineTo(new Vector2d(40,-38),0)
                .build();
        drive.followTrajectory(trajectory);
        drive.setPoseEstimate(new Pose2d(40,-38));
        //drive.setPoseEstimate(new Pose2d(90, 63));
        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d(40,-38),true)
                .splineTo(new Vector2d(-40,-63),Math.toRadians(180))
                .build();
        drive.followTrajectory(trajectory1);

        sleep(2000);
        /**
        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d(-28,-38,Math.PI/2),false)

                //.splineToConstantHeading(new Pose2d(45,-39,Math.toRadians(0)))
                //.strafeRight(70, DriveConstants.BASE_CONSTRAINTS)
                //.strafeTo(new Vector2d(40,-42),DriveConstants.BASE_CONSTRAINTS)
                .splineTo(new Vector2d(-5,-40),Math.toRadians(0))
                //.lineTo(new Vector2d(25,-40))
                .splineToConstantHeading(new Vector2d(25,-40),Math.toRadians(0))
                .splineTo(new Vector2d(40,-40),Math.toRadians(90))
                .build();
        drive.followTrajectory(trajectory1);
         */

    }
}

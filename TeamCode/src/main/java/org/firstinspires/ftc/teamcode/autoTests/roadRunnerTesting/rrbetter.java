package org.firstinspires.ftc.teamcode.autoTests.roadRunnerTesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Ladle;
import org.firstinspires.ftc.teamcode.roadRunnerTuner.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "test path", group = "drive")
public class rrbetter extends LinearOpMode {

    Ladle robo = new Ladle();
    final double upServo = .52;
    final double bottomServo = .83;

    @Override

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robo.init(hardwareMap);
        robo.imu();

        Pose2d startPose = new Pose2d(-62.0, -18.0, 0.0);
        drive.setPoseEstimate(startPose);


        waitForStart();

        if (isStopRequested()) return;

        robo.pusher.setPosition(bottomServo);

        sleep(1000);

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .forward(40.0)
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(-50.0, -36.0), 0.0)
                        .build()
        );

        sleep(1000);

        robo.launcher.setPower(1);

        sleep(1000);

        for (int i = 0; i <= 5; i++) {
            robo.pusher.setPosition(upServo);
            sleep(400);
            robo.pusher.setPosition(bottomServo);
        }

        sleep(200);

        robo.launcher.setPower(0);

        sleep(500);
    }
}
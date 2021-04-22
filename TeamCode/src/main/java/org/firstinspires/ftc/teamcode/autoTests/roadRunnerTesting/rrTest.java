package org.firstinspires.ftc.teamcode.autoTests.roadRunnerTesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Ladle;
import org.firstinspires.ftc.teamcode.roadRunnerTuner.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "rrTest", group = "drive")
public class rrTest extends LinearOpMode {

    Ladle robo = new Ladle();

    final double upServo = .52;
    final double bottomServo = .83;

    boolean servoMoving = false;

    public ElapsedTime launchTime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(-62.0, -18.0, 0.0);
        drive.setPoseEstimate(startPose);


        robo.init(hardwareMap);
        robo.imu();


        launchTime.reset();

        waitForStart();

        if (isStopRequested()) return;


        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-62.0, -18.0))
                .forward(40.0)
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(-0.0, -36.0), 0.0)
                        .build()
        );


//        sleep(2000);
//
//
//        robo.launcher.setPower(1);
//
//        sleep(300);
//
//        for (int i = 0; i <= 3; i++) {
//            boolean cycle = false;
//
//            while (!cycle) {
//                if (launchTime.milliseconds() >= 300 && !servoMoving) {
//                    robo.pusher.setPosition(upServo);
//                    servoMoving = true;
//                    launchTime.reset();
//                }
//                if (launchTime.milliseconds() >= 300 && servoMoving) {
//                    robo.pusher.setPosition(bottomServo);
//                    servoMoving = false;
//                    launchTime.reset();
//                    cycle = true;
//                }
//            }
//        }
//
//        robo.launcher.setPower(0);



    }
}
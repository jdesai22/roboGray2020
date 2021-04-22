package org.firstinspires.ftc.teamcode.autoTests.roadRunnerTesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Ladle;
import org.firstinspires.ftc.teamcode.roadRunnerTuner.drive.SampleMecanumDrive;

@Autonomous(name = "better rr autopaths!?", group = "Important")
public class AutoPathsv2 extends LinearOpMode {

    Ladle robo = new Ladle();

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        check if heading should be 180 or 0, dont want rotation in first step
        Pose2d startPose = new Pose2d(-62.0, -18.0, 0.0);
        drive.setPoseEstimate(startPose);

        int RingNumber = 0;

        robo.init(hardwareMap);
        robo.imu();

        waitForStart();

        while(!opModeIsActive()) {

            telemetry.update();
        }

        if (opModeIsActive()){


//            detect rings

//            Trajectory AvoidRings = drive.trajectoryBuilder(new Pose2d())
//                    .strafeLeft(5.0)
//                    .build();

            Trajectory MoveToLaunchArea = drive.trajectoryBuilder(startPose, true)
                    .forward(40.0)
                    .splineTo(new Vector2d(0.0, -40.0), 0.0)
                    .build();


//            move to correct zone

            Trajectory ZoneA = drive.trajectoryBuilder(MoveToLaunchArea.end(), true)
                    .splineTo(new Vector2d(15.0, -40.0), 0.0)
                    .build();

            Trajectory ZoneB = drive.trajectoryBuilder(MoveToLaunchArea.end(), true)
                    .splineTo(new Vector2d(43.0, -20.0), 0.0)
                    .build();

            Trajectory ZoneC = drive.trajectoryBuilder(MoveToLaunchArea.end(), true)
                    .splineTo(new Vector2d(62.0, -40.0), 0.0)
                    .build();



//            return to 2nd wobble goal and if possible collect other rings

            Trajectory ZoneAReturn = drive.trajectoryBuilder(ZoneA.end())
                    .splineTo(new Vector2d(0.0, -40.0), 0.0)
                    .splineTo(new Vector2d(-50.0, -35.0), 0.0)
                    .build();


//            SEE IF WE CAN COMBINE THESE ZONEBRETURNS INTO ONE OR IF DISCONTINUITIES OCCUR

            Trajectory ZoneBReturn1 = drive.trajectoryBuilder(ZoneB.end())
                    .splineTo(new Vector2d(30.0, -20.0), 0.0)
                    .splineTo(new Vector2d(0.0, -37.0), 0.0)
                    .build();

//            Trajectory ZoneBReturn2 = drive.trajectoryBuilder(ZoneBReturn1.end())
//                    .strafeRight(17.0)
//                    .build();

//            slow this down to pick up rings
            Trajectory ZoneBReturn3 = drive.trajectoryBuilder(ZoneBReturn1.end())
                    .back(30.0)  // is this right direciton even w/reversed = true
                    .splineTo(new Vector2d(-50.0, -35.0), 0.0)
                    .build();



            Trajectory ZoneCReturn = drive.trajectoryBuilder(ZoneC.end())
                    .splineTo(new Vector2d(0.0, -37.0), 0.0)
//            .back(30.0)
//                    pick up rings
                    .splineTo(new Vector2d(-30.0, -37.0), 0.0)
                    .build();

//            shoot rings
            Trajectory ZoneCReturn1 = drive.trajectoryBuilder(ZoneCReturn.end())
                    .splineTo(new Vector2d(0.0, -40.0), 0.0)
                    .build();

//            pick up final ring
            Trajectory ZoneCReturn2 = drive.trajectoryBuilder(ZoneCReturn1.end())
                    .splineTo(new Vector2d(-50.0, -35.0), 0.0)
                    .build();


//          pick up 2nd wobble goal and go to deposit
            Trajectory ZoneA1 = drive.trajectoryBuilder(ZoneAReturn.end(), true)
                    .splineTo(new Vector2d(10.0, -40.0), 0.0)
                    .build();

            Trajectory ZoneB1 = drive.trajectoryBuilder(ZoneBReturn3.end(), true)
                    .splineTo(new Vector2d(0.0, -35.0), 0.0)
                    .build();
//           shoot 1 ring on the way

            Trajectory ZoneB2 = drive.trajectoryBuilder(ZoneB1.end(), true)
                    .splineTo(new Vector2d(36.0, -20.0), 0.0)
                    .build();

            Trajectory ZoneC1 = drive.trajectoryBuilder(ZoneCReturn2.end(), true)
                    .splineTo(new Vector2d(55.0, -40.0), 0.0)
                    .build();

//          deposit wobble goal
//          park from zone
            Trajectory ZoneBPark = drive.trajectoryBuilder(ZoneB2.end())
                    .forward(20.0)
                    .build();

            Trajectory ZoneCPark = drive.trajectoryBuilder(ZoneC1.end())
                    .forward(40.0)
                    .build();


//            compile trajectories

//            drive.followTrajectory(AvoidRings);
            telemetry.addData("Status: ", "Move to Launch Zone");
            drive.followTrajectory(MoveToLaunchArea);

            switch (RingNumber) {
                case (0): {
                    telemetry.addData("Status: ", "To Zone A Wobble");
                    drive.followTrajectory(ZoneA);
                    telemetry.addData("Status: ", "Return from Zone A");
                    drive.followTrajectory(ZoneAReturn);
                    telemetry.addData("Status: ", "Deposit 2nd Wobble Goal & Park");
                    drive.followTrajectory(ZoneA1);
                }
                case (1): {
                    telemetry.addData("Status: ", "To Zone B Wobble");
                    drive.followTrajectory(ZoneB);
                    drive.followTrajectory(ZoneBReturn1);
//                    drive.followTrajectory(ZoneBReturn2);
                    drive.followTrajectory(ZoneBReturn3);
                    drive.followTrajectory(ZoneB1);
                    drive.followTrajectory(ZoneB2);
                    drive.followTrajectory(ZoneBPark);
                }
                case (4): {
                    drive.followTrajectory(ZoneC);
                    drive.followTrajectory(ZoneCReturn);
                    drive.followTrajectory(ZoneCReturn1);
                    drive.followTrajectory(ZoneCReturn2);
                    drive.followTrajectory(ZoneC1);
                    drive.followTrajectory(ZoneCPark);
                }
            }


            telemetry.addData("Back Left", robo.backLeft.getCurrentPosition());
            telemetry.addData("Front Left", robo.frontLeft.getCurrentPosition());
            telemetry.addData("Back Right", robo.backRight.getCurrentPosition());
            telemetry.addData("Front Right", robo.frontRight.getCurrentPosition());

            telemetry.update();
        }

    }



    private double convertRad(double degree) {
        return (Math.toRadians(degree));
    }

}

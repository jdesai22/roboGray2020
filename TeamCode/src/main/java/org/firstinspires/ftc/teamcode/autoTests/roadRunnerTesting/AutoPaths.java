package org.firstinspires.ftc.teamcode.autoTests.roadRunnerTesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Ladle;
import org.firstinspires.ftc.teamcode.roadRunnerTuner.drive.SampleMecanumDrive;

@Disabled
@Autonomous(name = "rr auto paths", group = "Important")
public class AutoPaths extends LinearOpMode {

    Ladle robo = new Ladle();

    enum State {
        Start,
        LaunchArea,
        Zone,
        Pickup,
        Zone2,
        Park,
    }

    State current = State.Start;

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(-62.0, -18.0, 0.0);

        int RingNumber = 0;

        robo.init(hardwareMap);
        robo.imu();

        waitForStart();

        if (isStopRequested()) return;



//            detect rings

//            Trajectory AvoidRings = drive.trajectoryBuilder(new Pose2d())
//                    .strafeLeft(5.0)
//                    .build();
        drive.setPoseEstimate(startPose);


            Trajectory MoveToLaunchArea = drive.trajectoryBuilder(startPose)
                    .forward(40.0)
                    .splineTo(new Vector2d(0.0, -40.0), 0.0)
                    .build();


//            move to correct zone

            Trajectory ZoneA = drive.trajectoryBuilder(MoveToLaunchArea.end())
                    .splineTo(new Vector2d(15.0, -40.0), 0.0)
                    .build();

            Trajectory ZoneB = drive.trajectoryBuilder(MoveToLaunchArea.end())
                    .splineTo(new Vector2d(43.0, -20.0), 0.0)
                    .build();

            Trajectory ZoneC = drive.trajectoryBuilder(MoveToLaunchArea.end())
                    .splineTo(new Vector2d(62.0, -40.0), 0.0)
                    .build();


//            return to 2nd wobble goal and if possible collect other rings

            Trajectory ZoneAReturn = drive.trajectoryBuilder(ZoneA.end(), true)
                    .splineTo(new Vector2d(0.0, -40.0), 0.0)
                    .splineTo(new Vector2d(-50.0, -35.0), 0.0)
                    .build();


//            SEE IF WE CAN COMBINE THESE ZONEBRETURNS INTO ONE OR IF DISCONTINUITIES OCCUR

            Trajectory ZoneBReturn1 = drive.trajectoryBuilder(ZoneB.end(), true)
                    .splineTo(new Vector2d(30.0, -20.0), 0.0)
                    .splineTo(new Vector2d(0.0, -37.0), 0.0)
                    .build();

//            Trajectory ZoneBReturn2 = drive.trajectoryBuilder(ZoneBReturn1.end())
//                    .strafeRight(17.0)
//                    .build();

//            slow this down to pick up rings
            Trajectory ZoneBReturn3 = drive.trajectoryBuilder(ZoneBReturn1.end(), true)
                    .back(30.0)  // is this right direciton even w/reversed = true
                    .splineTo(new Vector2d(-50.0, -35.0), 0.0)
                    .build();



            Trajectory ZoneCReturn = drive.trajectoryBuilder(ZoneC.end(), true)
                    .splineTo(new Vector2d(0.0, -37.0), 0.0)
//            .back(30.0)
//                    pick up rings
                    .splineTo(new Vector2d(-30.0, -37.0), 0.0)
                    .build();

//            shoot rings
            Trajectory ZoneCReturn1 = drive.trajectoryBuilder(ZoneCReturn.end(), true)
                    .splineTo(new Vector2d(0.0, -40.0), 0.0)
                    .build();

//            pick up final ring
            Trajectory ZoneCReturn2 = drive.trajectoryBuilder(ZoneCReturn1.end(), true)
                    .splineTo(new Vector2d(-50.0, -35.0), 0.0)
                    .build();


//          pick up 2nd wobble goal and go to deposit
            Trajectory ZoneA1 = drive.trajectoryBuilder(ZoneAReturn.end())
                    .splineTo(new Vector2d(10.0, -40.0), 0.0)
                    .build();

            Trajectory ZoneB1 = drive.trajectoryBuilder(ZoneBReturn3.end())
                    .splineTo(new Vector2d(0.0, -35.0), 0.0)
                    .build();
//           shoot 1 ring on the way

            Trajectory ZoneB2 = drive.trajectoryBuilder(ZoneB1.end())
                    .splineTo(new Vector2d(36.0, -20.0), 0.0)
                    .build();

            Trajectory ZoneC1 = drive.trajectoryBuilder(ZoneCReturn2.end())
                    .splineTo(new Vector2d(55.0, -40.0), 0.0)
                    .build();

//          deposit wobble goal
//          park from zone
            Trajectory ZoneBPark = drive.trajectoryBuilder(ZoneB2.end())
                    .back(20.0)
                    .build();

            Trajectory ZoneCPark = drive.trajectoryBuilder(ZoneC1.end())
                    .back(40.0)
                    .build();


//            compile trajectories
            
//            drive.followTrajectory(AvoidRings);

//            sleep(1000);
//            drive.followTrajectory(MoveToLaunchArea);
//            sleep(1000);

//            switch (RingNumber) {
//                case (0): {
//                    drive.followTrajectory(ZoneA);
//                    sleep(1000);
//                    drive.followTrajectory(ZoneAReturn);
//                    sleep(1000);
//                    drive.followTrajectory(ZoneA1);
//                    sleep(1000);
//                }
//                case (1): {
//                    drive.followTrajectory(ZoneB);
//                    drive.followTrajectory(ZoneBReturn1);
////                    drive.followTrajectory(ZoneBReturn2);
//                    drive.followTrajectory(ZoneBReturn3);
//                    drive.followTrajectory(ZoneB1);
//                    drive.followTrajectory(ZoneB2);
//                    drive.followTrajectory(ZoneBPark);
//                }
//                case (4): {
//                    drive.followTrajectory(ZoneC);
//                    drive.followTrajectory(ZoneCReturn);
//                    drive.followTrajectory(ZoneCReturn1);
//                    drive.followTrajectory(ZoneCReturn2);
//                    drive.followTrajectory(ZoneC1);
//                    drive.followTrajectory(ZoneCPark);
//                }
//            }
            drive.followTrajectoryAsync(MoveToLaunchArea);

            switch (current) {
                case Start:
                    if(!drive.isBusy()) {
                        current = State.Zone;
                        drive.followTrajectoryAsync(ZoneA);
                    }
                    break;
                case Zone:
                    if(!drive.isBusy()) {
                        current = State.Park;
                        drive.followTrajectoryAsync(ZoneAReturn);
                    }
                    break;
                case Park:
                    break;
            }

            telemetry.addData("Back Left", robo.backLeft.getCurrentPosition());
            telemetry.addData("Front Left", robo.frontLeft.getCurrentPosition());
            telemetry.addData("Back Right", robo.backRight.getCurrentPosition());
            telemetry.addData("Front Right", robo.frontRight.getCurrentPosition());

            telemetry.update();
        }


    private double convertRad(double degree) {
        return (Math.toRadians(degree));
    }

}
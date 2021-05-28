package org.firstinspires.ftc.teamcode.autoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Ladle;
import org.firstinspires.ftc.teamcode.roadRunnerTuner.drive.SampleMecanumDrive;

@Autonomous(group = "advanced")
public class fsmAutoZoneC extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        Start,
        LaunchArea,
        SecondLaunch,
        LaunchArea2,
        LaunchArea3,
        Shoot,
        DropGoal1,
        DropGoal2,
        ZoneC,
        ZoneC2,
        ZoneC3,
        ContinueToPickup,
        FinalPickup,
        StartIntake,
        StartIntake2,
        StopIntake,
        StopIntake2,
        RaiseGoal,
        Park,
        Parked
    }

    // We define the current state we're on
    State currentState = State.Start;

    // Define our start pose
    Pose2d startPose = new Pose2d(-62.0, -18.0, 0.0);

    final double upServo = .48;
    final double bottomServo = .83;

    boolean servoMoving = false;

    public ElapsedTime launchTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Ladle robo = new Ladle();
        robo.autoInit(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories

//        check if need to break this into two diff trajectories

        Trajectory ZoneC = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(60.0, -43.0), 0.0)
                .build();

        Trajectory MoveToLaunchArea = drive.trajectoryBuilder(ZoneC.end(), true)
//                .forward(40.0)
//                .splineTo(new Vector2d(30.0, -20.0), Math.toRadians(180))
                .splineTo(new Vector2d(-1.0, -36.0), Math.toRadians(180))
                .build();

        Trajectory ZoneCReturn = drive.trajectoryBuilder(MoveToLaunchArea.end(), true)
                //            .back(30.0)
//                    pick up rings
                .splineTo(new Vector2d(-25.0, -37.0), Math.toRadians(180))
                .build();

//            shoot rings
        Trajectory ZoneCReturn1 = drive.trajectoryBuilder(ZoneCReturn.end())
                .splineTo(new Vector2d(-1.0, -36.0), 0.0)
                .build();

//            pick up final ring
        Trajectory ZoneCReturn2 = drive.trajectoryBuilder(ZoneCReturn1.end(), true)
                .splineTo(new Vector2d(-45.0, -35.0), Math.toRadians(180))
                .build();

        Trajectory ZoneC1 = drive.trajectoryBuilder(ZoneCReturn2.end())
                .splineTo(new Vector2d(-1.0, -36.0), 0.0)
                .build();

        Trajectory ZoneC2 = drive.trajectoryBuilder(ZoneC1.end())
                .splineTo(new Vector2d(56.0, -40.0), 0.0)
                .build();

        Trajectory ZoneCPark = drive.trajectoryBuilder(ZoneC2.end())
                .back(40.0)
                .build();

        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.DropGoal1;
        drive.followTrajectoryAsync(ZoneC);


        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {


//                    break;

                case DropGoal1:

                    if (!drive.isBusy()) {
                        sleep(500);
                        dropWobble(robo);
                        sleep(500);
                        robo.launcher.setPower(1);

                        currentState = State.LaunchArea;
                    }
                    break;


                case LaunchArea:

                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(MoveToLaunchArea);
                        currentState = State.Shoot;
                    }

                    break;

                case Shoot:

                    if (!drive.isBusy()) {
                        shootRings(robo, 4);
//                        sleep(250);
                        resetDown(robo);
                        currentState = State.StartIntake;
                    }

                    break;
                case StartIntake:
                    if (!drive.isBusy()) {
                        robo.intake1.setPower(-1);
                        robo.intake2.setPower(-1);
//                        sleep(250);
                        currentState = State.ContinueToPickup;
                    }
                    break;

                case ContinueToPickup:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(ZoneCReturn);
                        currentState = State.SecondLaunch;
                    }
                    break;

                case SecondLaunch:

                    if (!drive.isBusy()) {
//                        robo.intake1.setPower(0);
//                        robo.intake2.setPower(0);

                        robo.launcher.setPower(1);
                        sleep(500);
                        drive.followTrajectoryAsync(ZoneCReturn1);
                        currentState = State.LaunchArea2;
                    }

                    break;

                case LaunchArea2:

                    if (!drive.isBusy()) {
                        shootRings(robo, 1);

                        currentState = State.StartIntake2;
                    }
                case StartIntake2:

                    if (!drive.isBusy()) {
                        robo.intake1.setPower(-1);
                        robo.intake2.setPower(-1);
                        drive.followTrajectoryAsync(ZoneCReturn2);
                        currentState = State.RaiseGoal;
                    }

                    break;

//                case StopIntake:
//                    if (!drive.isBusy()) {
//                        robo.intake1.setPower(0);
//                        robo.intake2.setPower(0);
////                        sleep(250);
//                        currentState = State.RaiseGoal;
//                    }
//                    break;

                case RaiseGoal:
                    if (!drive.isBusy()) {
//                        robo.intake1.setPower(0);
//                        robo.intake2.setPower(0);

                        sleep(250);

//                        raiseWobble(robo);
                        robo.grasp.setPosition(0);

                        robo.launcher.setPower(1);

                        currentState = State.ZoneC2;
//                        sleep(250);
                    }

                    break;
//
                case ZoneC2:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(ZoneC1);
//                        robo.launcher.setPower(1);
                        currentState = State.LaunchArea3;
                    }
                    break;
////
                case LaunchArea3:
                    if (!drive.isBusy()) {
                        robo.intake1.setPower(0);
                        robo.intake2.setPower(0);
                        shootRings(robo, 4);
                        currentState = State.ZoneC3;
                    }

                    break;

                case ZoneC3:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(ZoneC2);
                        currentState = State.DropGoal2;
                    }
                    break;

                case DropGoal2:
                    if (!drive.isBusy()) {
//                        sleep(500);
//                        dropWobble(robo);
//                        sleep(500);

                        robo.grasp.setPosition(.5);
                        currentState = State.Park;
                    }
                    break;
                case Park:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(ZoneCPark);
                        currentState = State.Parked;
                    }
                    break;
                case Parked:
                    break;

            }

            // Anything outside of the switch statement will run independent of the currentState
            drive.update();


            telemetry.update();
        }
    }

    public void shootRings(Ladle robo, double shots) {

        launchTime.reset();

        robo.launcher.setPower(1);

//        sleep(250);

        for (int i = 0; i < shots; i++) {
            boolean cycle = false;

            while (!cycle) {
                if (launchTime.milliseconds() >= 350 && !servoMoving) {
                    robo.pusher.setPosition(upServo);
                    servoMoving = true;
                    launchTime.reset();
                }
                if (launchTime.milliseconds() >= 350 && servoMoving) {
                    robo.pusher.setPosition(bottomServo);
                    servoMoving = false;
                    launchTime.reset();
                    cycle = true;
                }
            }
        }

        robo.launcher.setPower(0);

    }

    public void raiseWobble(Ladle robo) {
//        robo.grasp.setPosition(0.5);
//        robo.arm.setPower(-.6);

//        sleep(500);

        robo.grasp.setPosition(0);

        sleep(500);
        robo.arm.setPower(-.65);
        sleep(650);
        robo.arm.setPower(0);

    }


    public void dropWobble(Ladle robo) {

//        robo.arm.setPower(.4);
//        sleep(1000);
//
//        robo.arm.setPower(0);
//        robo.grasp.setPosition(0.5);
//
//        sleep(500);
//        robo.arm.setPower(0);

//        sleep(500);


//        sleep(500);


        robo.arm.setPower(.5);
        sleep(600);

        robo.arm.setPower(0);
        robo.grasp.setPosition(0.5);

        sleep(700);

        resetUp(robo);

    }

    public void resetUp(Ladle robo) {
        robo.arm.setPower(-.65);
        sleep(350);
        robo.arm.setPower(0);
    }

    public void resetDown(Ladle robo) {
        robo.arm.setPower(.65);
        sleep(500);
        robo.arm.setPower(0);
    }
}
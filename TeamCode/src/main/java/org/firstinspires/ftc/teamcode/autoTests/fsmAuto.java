package org.firstinspires.ftc.teamcode.autoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Ladle;
import org.firstinspires.ftc.teamcode.roadRunnerTuner.drive.SampleMecanumDrive;

@Autonomous(name = "fsmAutoZone1", group = "advanced")
public class fsmAuto extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        Start,
        LaunchArea,
        Shoot,
        DropGoal1,
        DropGoal2,
        ZoneA,
        ZoneA2,
        Pickup,
        RaiseGoal,
        Park
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.Start;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
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
        Trajectory MoveToLaunchArea = drive.trajectoryBuilder(startPose)
                .forward(40.0)
                .splineTo(new Vector2d(-1.0, -36.0), 0.0)
                .build();

        Trajectory ZoneA = drive.trajectoryBuilder(MoveToLaunchArea.end())
                .splineTo(new Vector2d(15.0, -40.0), 0.0)
                .build();

        Trajectory ZoneAReturn = drive.trajectoryBuilder(ZoneA.end(), true)
//                .splineTo(new Vector2d(0.0, -40.0), 0.0)
                .splineTo(new Vector2d(-45.0, -35.0), Math.toRadians(180))
                .build();

        Trajectory ZoneA1 = drive.trajectoryBuilder(ZoneAReturn.end())
                .splineTo(new Vector2d(10.0, -40.0), 0.0)
                .build();


        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.Shoot;
        drive.followTrajectoryAsync(MoveToLaunchArea);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {


//                    break;


                case Shoot:

                    if (!drive.isBusy()) {
                        shootRings(robo);
                        sleep(500);
                        currentState = State.LaunchArea;
                    }

                    break;

                case LaunchArea:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(ZoneA);
                        currentState = State.DropGoal1;
                    }

                    break;

                case DropGoal1:


                    if (!drive.isBusy()) {

                        sleep(500);

                        dropWobble(robo);

                        sleep(750);

                        currentState = State.ZoneA;
                    }
                    break;

                case ZoneA:
                    // Check if the drive class is busy following the trajectory

                    if (!drive.isBusy()) {
                        resetDown(robo);
                        drive.followTrajectoryAsync(ZoneAReturn);
                        currentState = State.RaiseGoal;
                    }

                    break;


//
                case RaiseGoal:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {



                        sleep(500);

                        raiseWobble(robo);

                        currentState = State.ZoneA2;

                        sleep(500);

                    }

                    break;
//
                case ZoneA2:

                    if (!drive.isBusy()) {
                        currentState = State.DropGoal2;
                        drive.followTrajectoryAsync(ZoneA1);
                    }


                    break;
////
                case DropGoal2:



                    if (!drive.isBusy()) {
                        sleep(500);

                        dropWobble(robo);

                        sleep(500);

                        currentState = State.Park;
                    }
                    break;

                case Park:
                    break;


            }

            // Anything outside of the switch statement will run independent of the currentState
            drive.update();


            telemetry.update();
        }
    }

    public void shootRings(Ladle robo) {

        launchTime.reset();

        robo.launcher.setPower(1);

        sleep(300);

        for (int i = 0; i <= 3; i++) {
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

    public void raiseWobbleOld(Ladle robo) {
        robo.grasp.setPosition(0);

        sleep(500);
        //neg power = drop
        robo.arm.setPower(-.55);
        sleep(700);
        robo.arm.setPower(0);

    }

    public void dropWobbleOld(Ladle robo) {

        robo.arm.setPower(.4);
        sleep(1000);


        robo.arm.setPower(0);
        robo.grasp.setPosition(0.5);


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

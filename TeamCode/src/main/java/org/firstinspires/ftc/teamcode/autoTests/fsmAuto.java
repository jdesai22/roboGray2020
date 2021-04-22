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

@Autonomous(group = "advanced")
public class fsmAuto extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        Start,
        LaunchArea,
        Shoot,
        ZoneA,
        Pickup,
        ZoneA2,
        Park
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.Start;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-62.0, -18.0, 0.0);

    final double upServo = .50;
    final double bottomServo = .83;

    boolean servoMoving = false;

    public ElapsedTime launchTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Ladle robo = new Ladle();
        robo.init(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories

//        check if need to break this into two diff trajectories
        Trajectory MoveToLaunchArea = drive.trajectoryBuilder(startPose)
                .forward(40.0)
                .splineTo(new Vector2d(0.0, -40.0), 0.0)
                .build();

        Trajectory ZoneA = drive.trajectoryBuilder(MoveToLaunchArea.end())
                .splineTo(new Vector2d(15.0, -40.0), 0.0)
                .build();

        Trajectory ZoneAReturn = drive.trajectoryBuilder(ZoneA.end(), true)
                .splineTo(new Vector2d(0.0, -40.0), 0.0)
                .splineTo(new Vector2d(-50.0, -35.0), 0.0)
                .build();

        Trajectory ZoneA1 = drive.trajectoryBuilder(ZoneAReturn.end())
                .splineTo(new Vector2d(10.0, -40.0), 0.0)
                .build();


        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.LaunchArea;
        drive.followTrajectoryAsync(MoveToLaunchArea);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case LaunchArea:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.ZoneA;
                        drive.followTrajectoryAsync(ZoneA);
                    }
                    break;

//                case Shoot:
//                    shootRings(robo);
//                    break;
                case ZoneA:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.Pickup;
                        drive.followTrajectoryAsync(ZoneAReturn);
                    }
                    break;
                case Pickup:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = State.ZoneA2;
                        drive.followTrajectoryAsync(ZoneA1);
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

        for (int i = 0; i <= 5; i++) {
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
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Ladle;
import org.firstinspires.ftc.teamcode.roadRunnerTuner.drive.SampleMecanumDrive;

@Autonomous
@Disabled
public class AutoPowershots extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        Start,
        Shoot1,
        Move1,
        Shoot2,
        Move2,
        Shoot3,
        Move3
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
        Trajectory FirstMove = drive.trajectoryBuilder(startPose)
                .strafeRight(15)
                .build();

        Trajectory SecondMove = drive.trajectoryBuilder(startPose)
                .strafeRight(5)
                .build();

        Trajectory ThirdMove = drive.trajectoryBuilder(startPose)
                .strafeRight(5)
                .build();



        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.Shoot1;
        robo.launcher.setPower(1);
        drive.followTrajectoryAsync(FirstMove);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.
            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {


                case Shoot1:




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


}

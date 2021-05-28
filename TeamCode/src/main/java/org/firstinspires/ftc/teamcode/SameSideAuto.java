package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autoTests.fsmAuto;
import org.firstinspires.ftc.teamcode.autoTests.fsmAutoZoneB;
import org.firstinspires.ftc.teamcode.autoTests.fsmAutoZoneC;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.firstinspires.ftc.teamcode.roadRunnerTuner.drive.SampleMecanumDrive;

import java.util.ArrayList;


@Autonomous(name = "updated same side auto", group = "Important")
@Disabled
public class SameSideAuto extends LinearOpMode {


    //CAMERA
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // ADJUST horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    private int cameraMonitorViewId;

    // ROADRUNNER AND VARIABLES
    Pose2d startPose = new Pose2d(-62.0, -34.0, Math.toRadians(180));

    final double upServo = .53;// originally .52
    final double bottomServo = .89;

    boolean servoMoving = false;

    public ElapsedTime launchTime = new ElapsedTime();


    //RR ENUMS

    // RR ENUMS

    enum StateA {
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

    enum StateB {
        Start,
        LaunchArea,
        SecondLaunch,
        Shoot,
        DropGoal1,
        DropGoal2,
        ZoneB,
        ZoneB2,
        ZoneB3,
        ContinueToPickup,
        StartIntake,
        StopIntake,
        RaiseGoal,
        Park,
        Parked
    }

    enum StateC {
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


    StateA currentStateA = StateA.Start;
    StateB currentStateB = StateB.Start;
    StateC currentStateC = StateC.Start;



    @Override
    public void runOpMode() throws InterruptedException{
        Ladle robo = new Ladle();
        robo.autoInit(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        // PATHS ADD ZONE C LATER

        // ZONE A

        Trajectory MoveToLaunchAreaA = drive.trajectoryBuilder(startPose, true)
//                .forward(40.0)
                .lineToSplineHeading(new Pose2d(-0.5, -34.0, Math.toRadians(180)))
                .build();

        Trajectory ZoneA = drive.trajectoryBuilder(MoveToLaunchAreaA.end(), true)
                .lineToSplineHeading(new Pose2d(19.0, -40.0, Math.toRadians(180)))
                .build();

        Trajectory ZoneAReturn = drive.trajectoryBuilder(ZoneA.end())
//                .splineTo(new Vector2d(0.0, -40.0), 0.0)
//                .splineTo(new Vector2d(-45.0, -36.5), Math.toRadians(180))
                .splineTo(new Vector2d(-45.0, -34.5), 0.0)
                .addDisplacementMarker(10, () -> {
                    resetDown(robo);
                })
                .build();

        Trajectory ZoneA1 = drive.trajectoryBuilder(ZoneAReturn.end(), true)
                .splineTo(new Vector2d(10.0, -40.0), Math.toRadians(180))
                .build();

        // ZONE B

        Trajectory MoveToLaunchAreaB = drive.trajectoryBuilder(startPose, true)
//                .forward(40.0)
                .lineToSplineHeading(new Pose2d(-37.0, -34.0, Math.toRadians(180)))
                .build();

        // shoot 3

        Trajectory LaunchAreaB2 = drive.trajectoryBuilder(MoveToLaunchAreaA.end(), true)
                .lineToSplineHeading(new Pose2d(-0.5, -34.0, Math.toRadians(180)))
                .build();

        Trajectory ZoneB = drive.trajectoryBuilder(LaunchAreaB2.end(), true)
                .lineToSplineHeading(new Pose2d(43.0, -20.0, Math.toRadians(180)))
                .build();

        Trajectory ZoneBReturn3 = drive.trajectoryBuilder(MoveToLaunchAreaB.end())

                .splineTo(new Vector2d(-45.0, -33.5), 0.0)
                .build();


        Trajectory ZoneB2 = drive.trajectoryBuilder(ZoneBReturn3.end(), true)
                .splineTo(new Vector2d(36.0, -20.0), Math.toRadians(180))
                .build();

        Trajectory ZoneBPark = drive.trajectoryBuilder(ZoneB2.end())
                .forward(20.0)
                .build();


        // ZONE C


        Trajectory MoveToLaunchAreaC = drive.trajectoryBuilder(startPose, true)
//                .forward(40.0)
//                .splineTo(new Vector2d(30.0, -20.0), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-37.0, -34.0, Math.toRadians(180)))
                .build();

        Trajectory MoveToLaunchAreaC2 = drive.trajectoryBuilder(MoveToLaunchAreaC.end(), true)
                .lineToSplineHeading(new Pose2d(-25.0, -34.0, Math.toRadians(180)))
                .build();

        Trajectory MoveToLaunchAreaC3 = drive.trajectoryBuilder(MoveToLaunchAreaC2.end(), true)
                .lineToSplineHeading(new Pose2d(-0.5, -34.0, Math.toRadians(180)))
                .build();

        Trajectory ZoneC = drive.trajectoryBuilder(MoveToLaunchAreaC.end(), true)
//                .splineTo(new Vector2d(60.0, -40.0), 0.0)
//                .splineTo(new Vector2d(50.0, -10.0), 0.0)
                .splineTo(new Vector2d(60.0, -45.0), Math.toRadians(180))
                .build();


//            pick up final ring
        Trajectory ZoneCReturn2 = drive.trajectoryBuilder(ZoneC.end())
                .splineTo(new Vector2d(-45.0, -34.5), 0.0)
                .build();
        

        Trajectory ZoneC2 = drive.trajectoryBuilder(ZoneCReturn2.end(), true)
                .splineTo(new Vector2d(56.0, -40.0), Math.toRadians(180))
                .build();

        Trajectory ZoneCPark = drive.trajectoryBuilder(ZoneC2.end())
                .back(40.0)
                .build();



        cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

//        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        camera.openCameraDevice();

        FtcDashboard.getInstance().startCameraStream(camera, 30);

        ElapsedTime init = new ElapsedTime();

        init.reset();

        while (init.seconds() < 2) {}

        String height = "" + pipeline.getHeight();

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            switch (height){
                case "ZERO":
                    currentStateA = StateA.Shoot;
                    robo.platformUp();
                    drive.followTrajectoryAsync(MoveToLaunchAreaA);

                    while (opModeIsActive() && !isStopRequested()) {
                        // Our state machine logic
                        // You can have multiple switch statements running together for multiple state machines
                        // in parallel. This is the basic idea for subsystems and commands.

                        // We essentially define the flow of the state machine through this switch statement
                        switch (currentStateA) {


//                    break;


                            case Shoot:

                                if (!drive.isBusy()) {
                                    shootRings(robo, 4);
                                    sleep(500);
                                    currentStateA = StateA.LaunchArea;
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
                                    currentStateA = StateA.DropGoal1;
                                }

                                break;

                            case DropGoal1:


                                if (!drive.isBusy()) {

                                    sleep(500);

                                    dropWobble(robo);

                                    sleep(750);

                                    currentStateA = StateA.ZoneA;
                                }
                                break;

                            case ZoneA:
                                // Check if the drive class is busy following the trajectory

                                if (!drive.isBusy()) {
                                    resetUp(robo);
                                    drive.followTrajectoryAsync(ZoneAReturn);
                                    currentStateA = StateA.RaiseGoal;
                                }

                                break;


//
                            case RaiseGoal:
                                // Check if the drive class is busy turning
                                // If not, move onto the next state, TRAJECTORY_3, once finished
                                if (!drive.isBusy()) {

//                                    resetDown(robo);

                                    sleep(250);

                                    raiseWobble(robo);

                                    currentStateA = StateA.ZoneA2;

                                    sleep(500);

                                }

                                break;
//
                            case ZoneA2:

                                if (!drive.isBusy()) {
                                    currentStateA = StateA.DropGoal2;
                                    drive.followTrajectoryAsync(ZoneA1);
                                }


                                break;
////
                            case DropGoal2:



                                if (!drive.isBusy()) {
                                    sleep(500);

                                    dropWobble(robo);

                                    sleep(500);

                                    currentStateA = StateA.Park;
                                }
                                break;

                            case Park:
                                break;


                        }

                        // Anything outside of the switch statement will run independent of the currentState
                        drive.update();


                        telemetry.update();
                    }

                case "ONE":

                    currentStateB = StateB.LaunchArea;
                    robo.platformUp();
                    drive.followTrajectoryAsync(MoveToLaunchAreaB);

                    while (opModeIsActive() && !isStopRequested()) {
                        switch (currentStateB) {
                            case LaunchArea:

                                if (!drive.isBusy()) {
                                    shootRings(robo, 4);

                                    currentStateB = StateB.ContinueToPickup;
                                }
                                break;

                            case ContinueToPickup:
                                // Check if the drive class isn't busy
                                // `isBusy() == true` while it's following the trajectory
                                // Once `isBusy() == false`, the trajectory follower signals that it is finished
                                // We move on to the next state
                                // Make sure we use the async follow function
                                if (!drive.isBusy()) {
                                    robo.platformDown();
                                    robo.intake1.setPower(-1);
                                    robo.intake2.setPower(1);
                                    drive.followTrajectoryAsync(LaunchAreaB2);
                                    currentStateB = StateB.SecondLaunch;
                                }
                                break;

                            case SecondLaunch:

                                if (!drive.isBusy()) {
                                    robo.intake1.setPower(0);
                                    robo.intake2.setPower(0);
                                    robo.platformUp();

                                    shootRings(robo, 2);
//                        sleep(250);
                                    currentStateB = StateB.ZoneB;
                                }

                                break;
                            case ZoneB:

                                if (!drive.isBusy()) {
                                    drive.followTrajectoryAsync(ZoneB);
                                    currentStateB = StateB.DropGoal1;
                                }
                                break;

                            case DropGoal1:

                                if (!drive.isBusy()) {
                                    sleep(500);

                                    dropWobble(robo);

                                    sleep(750);
                                    currentStateB = StateB.ZoneB2;
                                }
                                break;


                            case ZoneB2:
                                if (!drive.isBusy()) {

                                    drive.followTrajectoryAsync(ZoneBReturn3);

                                    currentStateB = StateB.RaiseGoal;
                                }

                                break;

                            case RaiseGoal:
                                if (!drive.isBusy()) {

                                    sleep(250);
                                    raiseWobble(robo);
                                    sleep(500);

                                    currentStateB = StateB.ZoneB3;
                                }
                                break;

                            case ZoneB3:

                                if (!drive.isBusy()) {
                                    drive.followTrajectoryAsync(ZoneB2);
                                    currentStateB = StateB.ZoneB3;
                                }
                                break;

                            case DropGoal2:
                                if (!drive.isBusy()) {
                                    sleep(500);
                                    zoneBDropWobble(robo);
                                    sleep(500);
                                    currentStateB = StateB.Park;
                                }
                                break;

                            case Park:
                                if (!drive.isBusy()) {
                                    drive.followTrajectoryAsync(ZoneBPark);
                                    currentStateB = StateB.Parked;
                                }
                                break;
                            case Parked:
                                break;

                        }

                        // Anything outside of the switch statement will run independent of the currentState
                        drive.update();


                        telemetry.update();
                    }

                case "FOUR":

                    currentStateC = StateC.Shoot;
                    robo.platformUp();
                    drive.followTrajectoryAsync(MoveToLaunchAreaC);

                    while (opModeIsActive() && !isStopRequested()) {

                        switch (currentStateC) {

                            case Shoot:

                                if (!drive.isBusy()) {
                                    shootRings(robo, 4);
//                        sleep(250);
                                    resetDown(robo);
                                    currentStateC = StateC.StartIntake;
                                }

                                break;


                            case LaunchArea:

                                if (!drive.isBusy()) {
                                    drive.followTrajectoryAsync(MoveToLaunchAreaC);
                                    currentStateC = StateC.Shoot;
                                }

                                break;

                            case StartIntake:
                                if (!drive.isBusy()) {
                                    robo.intake1.setPower(-1);
                                    robo.intake2.setPower(-1);
//                        sleep(250);
                                    currentStateC = StateC.ContinueToPickup;
                                }
                                break;

                            case ContinueToPickup:
                                if (!drive.isBusy()) {
//                                    drive.followTrajectoryAsync(ZoneCReturn);
                                    currentStateC = StateC.SecondLaunch;
                                }
                                break;

                            case SecondLaunch:

                                if (!drive.isBusy()) {
//                        robo.intake1.setPower(0);
//                        robo.intake2.setPower(0);

                                    robo.launcher.setPower(0);
//                                    sleep(500);
//                                    drive.followTrajectoryAsync(ZoneCReturn1);
                                    currentStateC = StateC.LaunchArea2;
                                }

                                break;

                            case LaunchArea2:

                                if (!drive.isBusy()) {
                                    robo.launcher.setPower(1);
                                    sleep(500);
                                    shootRings(robo, 1);
                                    robo.launcher.setPower(0);
                                    currentStateC = StateC.StartIntake2;
                                }
                            case StartIntake2:

                                if (!drive.isBusy()) {
                                    robo.intake1.setPower(-1);
                                    robo.intake2.setPower(-1);
                                    drive.followTrajectoryAsync(ZoneCReturn2);
                                    currentStateC = StateC.RaiseGoal;
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

//                                    robo.launcher.setPower(1);

                                    currentStateC = StateC.ZoneC2;
//                        sleep(250);
                                }

                                break;

                            case DropGoal1:

                                if (!drive.isBusy()) {
                                    sleep(500);
                                    dropWobble(robo);
                                    sleep(500);
                                    robo.launcher.setPower(1);

                                    currentStateC = StateC.LaunchArea;
                                }
                                break;

//
                            case ZoneC2:
                                if (!drive.isBusy()) {
//                                    drive.followTrajectoryAsync(ZoneC1);
//                        robo.launcher.setPower(1);
                                    currentStateC = StateC.LaunchArea3;
                                }
                                break;
////
                            case LaunchArea3:
                                if (!drive.isBusy()) {
                                    robo.launcher.setPower(1);
                                    sleep(500);
                                    shootRings(robo, 4);
                                    robo.intake1.setPower(0);
                                    robo.intake2.setPower(0);
                                    currentStateC = StateC.ZoneC3;
                                }

                                break;

                            case ZoneC3:
                                if (!drive.isBusy()) {
                                    drive.followTrajectoryAsync(ZoneC2);
                                    currentStateC = StateC.DropGoal2;
                                }
                                break;

                            case DropGoal2:
                                if (!drive.isBusy()) {
//                        sleep(500);
//                        dropWobble(robo);
//                        sleep(500);

                                    robo.grasp.setPosition(.5);
                                    resetUp(robo);
                                    currentStateC = StateC.Park;
                                }
                                break;
                            case Park:
                                if (!drive.isBusy()) {
                                    drive.followTrajectoryAsync(ZoneCPark);
                                    currentStateC = StateC.Parked;
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

//    public void raiseWobble(Ladle robo) {
//        robo.grasp.setPosition(0);
//
//        sleep(500);
//        //neg power = drop
//        robo.arm.setPower(.65);
//        sleep(700);
//        robo.arm.setPower(0);
//
//    }
//
//    public void dropWobble(Ladle robo) {
//
//        robo.arm.setPower(.5);
//        sleep(1000);
//
//        robo.arm.setPower(0);
//        robo.grasp.setPosition(0.5);
//
//        sleep(500);
//
//        robo.arm.setPower(-.4);
//        sleep(500);
//        robo.arm.setPower(0);
//
//
//    }

    public void raiseWobble(Ladle robo) {
//        robo.grasp.setPosition(0.5);
//        robo.arm.setPower(-.6);

//        sleep(500);

        robo.grasp.setPosition(0);

        sleep(500);
        robo.arm.setPower(-.8);
        sleep(500);
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


        robo.arm.setPower(.7);
        sleep(500);

        robo.arm.setPower(0);
        sleep(150);

        robo.grasp.setPosition(0.5);
        sleep(100);

        resetUp(robo);

    }

    public void zoneBDropWobble(Ladle robo) {

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


        robo.arm.setPower(.65);
        sleep(450);

        robo.arm.setPower(0);
        sleep(300);

        robo.grasp.setPosition(0.5);
        sleep(100);

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

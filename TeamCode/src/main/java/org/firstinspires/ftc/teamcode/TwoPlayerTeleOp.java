package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roadRunnerTuner.drive.SampleMecanumDrive;

@TeleOp(name="TwoPlayerTeleOp", group="main")
@Disabled
public class TwoPlayerTeleOp extends LinearOpMode {
    Ladle robo = new Ladle();
    SampleMecanumDrive drive;

    //    for controller 1
    ToggleMap toggleMap1 = new ToggleMap();
    UseMap useMap1 = new UseMap();

    //    for controller 2
    ToggleMap toggleMap2 = new ToggleMap();
    UseMap useMap2 = new UseMap();
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    /* V * A * R * I * A * B * E * S *////* V * A * R * I * A * B * E * S */
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    private ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime launchTime = new ElapsedTime();

    //    VARIABLES
    boolean launcherRunning = false;
    boolean servoMoving = false;

    //    tune these constants
    final int launchBuffer = 200;
    final double upServo = .48;// originally .52
    final double bottomServo = .83;
    final boolean armOverride = false;

    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robo.autoInit(hardwareMap);

//        robo.imu();
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        robo.runWithoutEncoderDrive();


        double startTime = 0;


        while(!opModeIsActive()) {

            resetEncoders();
            telemetry.update();
        }

        startTime = runtime.milliseconds();
        robo.pusher.setPosition(bottomServo);
        launchTime.reset();

        waitForStart();

//        is the following necessary?
        if (isStopRequested()) return;

        while(opModeIsActive()){
            updateKeys();
//            angleOverflow();

            if (gamepad1.dpad_left) {

                powershots();

            }  else if (gamepad2.dpad_right) {

                setPoseRightWall();

            } else if (gamepad1.dpad_up) {

                autoAim();

            } else if (gamepad2.dpad_down) {

                setShootPose();
            }

            else {


                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                //see if this last heading/rotation works
                                -gamepad1.right_stick_x
                        )
                );

                drive.update();

            }

            push();
            launch();
            intake();
            clawClose();
            controlArm();

            telemetry.update();

//            resetEncoders();



//            telemetry.addData("Back Left", robo.backLeft.getCurrentPosition());
//            telemetry.addData("Front Left", robo.frontLeft.getCurrentPosition());
//            telemetry.addData("Back Right", robo.backRight.getCurrentPosition());
//            telemetry.addData("Front Right", robo.frontRight.getCurrentPosition());

//            telemetry.update();
        }
    }


    public void push() {


        if (gamepad1.right_bumper && launchTime.milliseconds() >= 300 && !servoMoving && toggleMap2.a) {
            robo.pusher.setPosition(upServo);
            servoMoving = true;
            launchTime.reset();
        }

        if (launchTime.milliseconds() >= 300 && servoMoving) {
            robo.pusher.setPosition(bottomServo);
            servoMoving = false;
            launchTime.reset();
        }

    }

    public void launch() {
//        check
        if(toggleMap2.a) {
            if (toggleMap1.left_bumper || toggleMap2.left_bumper) {
                robo.launcher.setPower(.75);
                telemetry.addData("Launcher:", "POWERSHOT MODE");
            } else {
                robo.launcher.setPower(1);
                telemetry.addData("Launcher:", "HIGH GOAL");
            }

        } else {
            robo.launcher.setPower(0);
            telemetry.addData("Launcher:", "REST");
        }


    }

    public void clawClose() {
        if(toggleMap1.b){
            robo.grasp.setPosition(.5);
            // closed
        }
        else {
            robo.grasp.setPosition(0);
            // open
        }
    }

    public void controlArm() {
        robo.arm.setPower(.8 * gamepad2.right_stick_y);
    }

    public void intake() {
//        check
        if (toggleMap2.x) {
            robo.intake1.setPower(1);
            robo.intake2.setPower(1);
            telemetry.addData("Intake:", "REVERSE");
        } else if (toggleMap2.y){
            robo.intake1.setPower(-1);
            robo.intake2.setPower(-1);
            telemetry.addData("Intake:", "NORMAL");
        } else {
            robo.intake1.setPower(0);
            robo.intake2.setPower(0);
            telemetry.addData("Intake:", "REST");

        }
    }

    public void resetEncoders(){
        if(gamepad2.y && gamepad2.b){
            if(!toggleMap2.b){
                useMap2.b = runtime.milliseconds(); //Using useMap weirdly here
            }
            toggleMap2.b = true; //Using toggleMap weirdly here
            if(cdCheck(useMap2.b, 2000)){
                telemetry.addData(">", "Encoders Successfully Reset");
            }
            else{
                telemetry.addData("WARNING", "RESETTING ENCODERS");
            }
        }
        else{
            toggleMap2.b = false;
        }
    }


    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    //[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[

    public void updateKeys(){
        if(gamepad1.left_bumper && cdCheck(useMap1.left_bumper, 500)){
            toggleMap1.left_bumper = toggle(toggleMap1.left_bumper);
            useMap1.left_bumper = runtime.milliseconds();
        }
        if(gamepad1.right_bumper && cdCheck(useMap1.right_bumper, 500)){
            toggleMap1.right_bumper = toggle(toggleMap1.right_bumper);
            useMap1.right_bumper = runtime.milliseconds();
//            toggleMap1.left_bumper = false;
        }
        if(gamepad1.b && cdCheck(useMap1.b, 500)){
            toggleMap1.b = toggle(toggleMap1.b);
            useMap1.b = runtime.milliseconds();
        }
        if(gamepad1.a && cdCheck(useMap1.a, 500)){
            toggleMap1.a = toggle(toggleMap1.a);
            useMap1.a = runtime.milliseconds();
        }
        if(gamepad1.x && cdCheck(useMap1.x, 500)){
            toggleMap1.x = toggle(toggleMap1.x);
            useMap1.x = runtime.milliseconds();
            toggleMap1.y = false;
        }
        if(gamepad1.y && cdCheck(useMap1.y, 500)){
            toggleMap1.y = toggle(toggleMap1.y);
            useMap1.y = runtime.milliseconds();
            toggleMap1.x = false;
        }
        if(gamepad2.left_bumper && cdCheck(useMap2.left_bumper, 500)){
            toggleMap2.left_bumper = toggle(toggleMap2.left_bumper);
            useMap2.left_bumper = runtime.milliseconds();
        }
        if (toggleMap1.right_bumper && cdCheck(useMap1.right_bumper, launchBuffer)) {
            toggleMap1.right_bumper = false;
            useMap1.right_bumper = runtime.milliseconds();
        }

        if (gamepad1.guide && cdCheck(useMap1.guide, 500)) {
            toggleMap1.guide = toggle(toggleMap1.guide);
            useMap1.guide = runtime.milliseconds();
        }
        if(gamepad2.b && cdCheck(useMap2.b, 500)){
            toggleMap2.b = toggle(toggleMap2.b);
            useMap2.b = runtime.milliseconds();
        }
        if(gamepad2.a && cdCheck(useMap2.a, 500)){
            toggleMap2.a = toggle(toggleMap2.a);
            useMap2.a = runtime.milliseconds();
        }
        if(gamepad2.x && cdCheck(useMap2.x, 500)){
            toggleMap2.x = toggle(toggleMap2.x);
            useMap2.x = runtime.milliseconds();
            toggleMap2.y = false;
        }
        if(gamepad2.y && cdCheck(useMap2.y, 500)){
            toggleMap2.y = toggle(toggleMap2.y);
            useMap2.y = runtime.milliseconds();
            toggleMap2.x = false;
        }

    }

    public void powershots() {
        robo.launcher.setPower(.75);

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // is this needed?
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        test this
        drive.setPoseEstimate(new Pose2d(-0.5, 5.0));

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(14.5)
                .addDisplacementMarker(18, () -> {
                    shootRings(robo, 1);
                    telemetry.addData("shoot ring", "1");
                })
//                .addTemporalMarker(1, ()->{})
                .build();


        drive.followTrajectory(traj1);

        sleep(400);

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(8)
                .addDisplacementMarker(() -> {
                    shootRings(robo, 1);

                })
                .build();

        drive.followTrajectory(traj2);

//        shootRings(robo, 1);

        sleep(400);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(5)
                .addDisplacementMarker(5, () -> {
                    shootRings(robo, 1);
                })
                .build();

        drive.followTrajectory(traj3);

//        shootRings(robo, 1);
//        sleep(400);

        robo.launcher.setPower(0);
    }

    public void autoAim() {
        Pose2d pose = drive.getPoseEstimate();
        Trajectory lineUp = drive.trajectoryBuilder(pose)
                .lineToSplineHeading(new Pose2d(-.5, -34, Math.toRadians(0.0)))
                .build();

        drive.followTrajectory(lineUp);
    }

    public void setPoseRightWall() {
        //fine tune
        drive.setPoseEstimate(new Pose2d(-0.5, -66));
    }

    public void setShootPose() {
        drive.setPoseEstimate(new Pose2d(-0.5, -34));
    }
//    public Pose2d getPose() {
//        return drive.getPoseEstimate();
//    }

    public void shootRings(Ladle robo, double shots) {

        launchTime.reset();

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
    }

    public boolean cdCheck(double key, int cdTime){
        return runtime.milliseconds() - key > cdTime;
    }


    public boolean toggle(boolean variable){
        if(variable == true){
            variable = false;
        }
        else if(variable == false){
            variable = true;
        }
        return variable;

    }

}

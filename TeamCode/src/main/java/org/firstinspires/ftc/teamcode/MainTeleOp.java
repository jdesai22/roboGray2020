package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MainTeleOp", group="main")
public class MainTeleOp extends LinearOpMode {
    Ladle robo = new Ladle();

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

    //int chungoidPos = 0;
    private ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime launchTime = new ElapsedTime();

//    VARIABLES
    boolean launcherRunning = false;
    boolean servoMoving = false;

    //    tune these constants
    final int launchBuffer = 200;
    final double upServo = .4;
    final double bottomServo = .657;
    final boolean armOverride = false;

    @Override
    public void runOpMode(){
        robo.init(hardwareMap);
        robo.imu();

        robo.runWithoutEncoderDrive();


        double startTime = 0;

        waitForStart();

        while(!opModeIsActive()) {

            resetEncoders();
            telemetry.update();
        }

        startTime = runtime.milliseconds();
        robo.pusher.setPosition(bottomServo);
        launchTime.reset();
        while(opModeIsActive()){
            updateKeys();
            drive();
            push();
            launch();
            intake();

            resetEncoders();

            if(!armOverride) {
                robo.clearArmVoltage();
            }

            telemetry.addData("Back Left", robo.backLeft.getCurrentPosition());
            telemetry.addData("Front Left", robo.frontLeft.getCurrentPosition());
            telemetry.addData("Back Right", robo.backRight.getCurrentPosition());
            telemetry.addData("Front Right", robo.frontRight.getCurrentPosition());

            telemetry.update();
        }
    }


    //Player 1
    public void drive() {
        double stick_x = -gamepad1.left_stick_x;
        double stick_y = gamepad1.left_stick_y;
        double pX = 0;
        double pY = 0;
        double pRot = 0;
        double rotMultiplier = 0.6;
        double theta = Math.atan2(stick_y, stick_x); //Arctan2 doesn't have bad range restriction

        if (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down) {

            double mag = 0.25;
            if (gamepad1.dpad_up) {
                pX = mag;
                pY = -mag;
            } else if (gamepad1.dpad_left) {
                pX = 2*mag;
                pY = 2*mag;
            } else if (gamepad1.dpad_down) {
                pX = -mag;
                pY = mag;
            } else if (gamepad1.dpad_right) {
                pX = -2*mag;
                pY = -2*mag;
            }

            pRot = -rotMultiplier*(gamepad1.right_trigger-gamepad1.left_trigger);
            robo.mecanumDrive(pX, pY, -pRot);
        }

        else {

            pRot = -0.6 * rotMultiplier*(gamepad1.right_trigger-gamepad1.left_trigger);

            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                pRot = -rotMultiplier*(gamepad1.right_trigger-gamepad1.left_trigger);
            }

            double gyroAngle = 0;
            double magnitudeMultiplier = 0;

//            ALL TRIG IS IN RADIANS
            double modifiedTheta = theta + Math.PI / 4 - gyroAngle;

            double thetaInFirstQuad = Math.abs(Math.atan(stick_y / stick_x)); //square to circle conversion
//            arctan is same as tan inverse

            if (thetaInFirstQuad > Math.PI / 4) {
                magnitudeMultiplier = Math.sin(thetaInFirstQuad); //Works because we know y is 1 when theta > Math.pi/4
            } else if (thetaInFirstQuad <= Math.PI / 4) {
                magnitudeMultiplier = Math.cos(thetaInFirstQuad); //Works because we know x is 1 when theta < Math.pi/4
            }

            double magnitude = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * magnitudeMultiplier * (1 - Math.abs(pRot)); //Multiplied by (1-pRot) so it doesn't go over 1 with rotating
            pX = magnitude * Math.cos(modifiedTheta);
            pY = magnitude * Math.sin(modifiedTheta);

            robo.mecanumDrive(pX, pY, -pRot);
        }

    }

    public void push() {
//        make sure toggle works
//        if(toggleMap1.right_bumper && toggleMap1.a){
//            robo.pusher.setPosition(upServo);
//        }
//        else {
//            robo.pusher.setPosition(bottomServo);
//        }

        if (gamepad1.right_bumper && launchTime.milliseconds() >= 150 && !servoMoving && toggleMap1.a) {
            robo.pusher.setPosition(upServo);
            servoMoving = true;
            launchTime.reset();
        }

        if (launchTime.milliseconds() >= 150 && servoMoving) {
            robo.pusher.setPosition(bottomServo);
            servoMoving = false;
            launchTime.reset();
        }

//        better version - need to test

//        if (launchTime.seconds() - launchTime.startTime() > launchBuffer && gamepad1.b && launcherRunning && robo.pusher.getPosition() == 0.5) {
//            robo.pusher.setPosition(0.75);
//            launchTime.reset();
//        } else if (launchTime.seconds() - launchTime.startTime() > launchBuffer && robo.pusher.getPosition() == 0.75){
//            robo.pusher.setPosition(0.5);
//        }

//        if (launchTime.seconds() - launchTime.startTime() > launchBuffer && gamepad1.right_trigger > 0.1 && launcherRunning && robo.pusher.getPosition() == 0.5) {
//            robo.pusher.setPosition(0.75);
//            launchTime.reset();
//        } else if (launchTime.seconds() - launchTime.startTime() > launchBuffer && robo.pusher.getPosition() == 0.75){
//            robo.pusher.setPosition(0.5);
//        }

//        if (launchTime.seconds() - launchTime.startTime() > launchBuffer && gamepad1.right_bumper && launcherRunning && robo.pusher.getPosition() == bottomServo) {
//            robo.pusher.setPosition(upServo);
//            launchTime.reset();
//            telemetry.addData("Pusher:", "UP");
//        } else if (launchTime.seconds() - launchTime.startTime() > launchBuffer && robo.pusher.getPosition() == upServo){
//            robo.pusher.setPosition(bottomServo);
//            telemetry.addData("Pusher:", "DOWN");
//        }
    }

    public void launch() {
//        check
        if(toggleMap1.a) {
            robo.launcher.setPower(1);
//            launcherRunning = true;
//            toggleMap1.x = false;
//            toggleMap1.y = false;
            telemetry.addData("Launcher:", "RUNNING");
        } else {
            robo.launcher.setPower(0);
//            launcherRunning = false;
            telemetry.addData("Launcher:", "REST");
        }
//        } else if (toggleMap1.b){
//            robo.launcher.setPower(0);
//            launcherRunning = false;
//            telemetry.addData("Launcher:", "STOPPED");
//        }

    }

    public void intake() {
//        check
        if (toggleMap1.x) {
            robo.intake1.setPower(1);
            robo.intake2.setPower(.55);
            telemetry.addData("Intake:", "NORMAL");
        } else if (toggleMap1.y){
            robo.intake1.setPower(-1);
            robo.intake2.setPower(-.55);
            telemetry.addData("Intake:", "REVERSE");
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
//        test
        if (toggleMap1.right_bumper && cdCheck(useMap1.right_bumper, launchBuffer)) {
            toggleMap1.right_bumper = false;
            useMap1.right_bumper = runtime.milliseconds();
        }


//        if(gamepad2.left_bumper){
//            toggleMap1.left_bumper = false;
//            toggleMap1.right_bumper = false;
//        }
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

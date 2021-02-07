package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="MainTeleOp", group="main")
public class MainTeleOp extends LinearOpMode {
    ladle robo = new ladle();

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


    @Override
    public void runOpMode(){
        robo.init(hardwareMap);
        robo.imu();

        robo.runWithoutEncoderDrive();
        //Check if slides hold position when transitioned
        double startTime = 0;

        waitForStart();

        while(!opModeIsActive()) {

            resetEncoders();
            telemetry.update();
        }

        startTime = runtime.milliseconds();
        while(opModeIsActive()){
            updateKeys();
            drive();
            push();
            launch();
            intake();

            resetEncoders();

            telemetry.addData("Push Pos", robo.pusher.getPosition());
            telemetry.addData("Launch Power", robo.launcher.getPower());
            telemetry.addData("Intake Power", robo.intake.getPower());
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
            robo.mecanumDrive(pX, pY, pRot);
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

            robo.mecanumDrive(pX, pY, pRot);
        }

    }

    public void push() {
//        make sure toggle works
        if(toggleMap1.b){
            robo.pusher.setPosition(1);
        }
        else {
            robo.pusher.setPosition(0.5);
        }
    }

    public void launch() {
//        check
        if(toggleMap1.a) {
            robo.launcher.setPower(1);
        } else if (toggleMap1.x){
            robo.launcher.setPower(0);
        }
    }

    public void intake() {
//        check
        if (toggleMap1.left_bumper) {
            robo.intake.setPower(1);
        } else if (toggleMap1.right_bumper){
            robo.intake.setPower(-1);
        } else {
            robo.intake.setPower(0);
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
            toggleMap1.right_bumper = false;
        }
        if(gamepad1.right_bumper && cdCheck(useMap1.right_bumper, 500)){
            toggleMap1.right_bumper = toggle(toggleMap1.right_bumper);
            useMap1.right_bumper = runtime.milliseconds();
            toggleMap1.left_bumper = false;
        }
        if(gamepad1.b && cdCheck(useMap1.b, 500)){
            toggleMap1.b = toggle(toggleMap1.b);
            useMap1.b = runtime.milliseconds();
            toggleMap1.b = false;
        }
        if(gamepad1.a && cdCheck(useMap1.a, 500)){
            toggleMap1.a = toggle(toggleMap1.a);
            useMap1.a = runtime.milliseconds();
            toggleMap1.a = false;
        }
        if(gamepad1.x && cdCheck(useMap1.x, 500)){
            toggleMap1.x = toggle(toggleMap1.x);
            useMap1.x = runtime.milliseconds();
            toggleMap1.x = false;
        }
        if(gamepad2.left_bumper){
            toggleMap1.left_bumper = false;
            toggleMap1.right_bumper = false;
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

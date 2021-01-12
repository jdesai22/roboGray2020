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

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.ToggleMap;
import org.firstinspires.ftc.teamcode.UseMap;


@TeleOp(name="DriveEncoderFinder", group="main")
public class DriveEncoderFinder extends LinearOpMode {
    Robot robo = new Robot();
    ToggleMap toggleMap1 = new ToggleMap();
    UseMap useMap1 = new UseMap();

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

        double startTime = 0;

        while(!opModeIsActive()){
            telemetry.update();
        }


        robo.resetDrive();

        if(toggleMap1.y){
            robo.runWithoutEncoderDrive();
        }
        else{
            robo.runToPosDrive();
        }

        startTime = runtime.milliseconds();


        while(opModeIsActive()){
            updateKeys();
            //Player 1
            if(toggleMap1.y){
                telemetry.addData("Drive", "Manual");
                drive();
            }
            else{
                telemetry.addData("Drive", "Encoder");
                encoderDrive(0.5);
            }
            reset();
            telemetry.addData("fl", robo.frontLeft.getCurrentPosition());
            telemetry.addData("bl", robo.backLeft.getCurrentPosition());
            telemetry.addData("br", robo.backRight.getCurrentPosition());
            telemetry.addData("fr", robo.frontRight.getCurrentPosition());
            telemetry.update();
        }

    }


    //Player 1
    int flPos = 0;
    int blPos = 0;
    int brPos = 0;
    int frPos = 0;


    public void encoderDrive(double power){
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

        } else {
            pRot = -0.6 * rotMultiplier*(gamepad1.right_trigger-gamepad1.left_trigger);
            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                pRot = -rotMultiplier*(gamepad1.right_trigger-gamepad1.left_trigger);
            }
            //double gyroAngle = getHeading(); //In radiants, proper rotation, yay!!11!!
            double gyroAngle = 0;
            double magnitudeMultiplier = 0;

            // if(!toggleMap1.left_bumper){ //Removes gyroAngle from the equation meaning the robot drives normally
            //     gyroAngle = 0;
            // }
            double modifiedTheta = theta + Math.PI / 4 - gyroAngle;

            double thetaInFirstQuad = Math.abs(Math.atan(stick_y / stick_x)); //square to circle conversion
            if (thetaInFirstQuad > Math.PI / 4) {
                magnitudeMultiplier = Math.sin(thetaInFirstQuad); //Works because we know y is 1 when theta > Math.pi/4
            } else if (thetaInFirstQuad <= Math.PI / 4) {
                magnitudeMultiplier = Math.cos(thetaInFirstQuad); //Works because we know x is 1 when theta < Math.pi/4
            }
            double magnitude = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * magnitudeMultiplier * (1 - Math.abs(pRot)); //Multiplied by (1-pRot) so it doesn't go over 1 with rotating
            pX = magnitude * Math.cos(modifiedTheta);
            pY = magnitude * Math.sin(modifiedTheta);
        }
        int bleh = 50;
        flPos += bleh*(pY + pRot);
        blPos += bleh*(pX - pRot);
        brPos += bleh*(pY - pRot);
        frPos += bleh*(pX + pRot);

        robo.frontLeft.setTargetPosition(flPos);
        robo.backLeft.setTargetPosition(blPos);
        robo.backRight.setTargetPosition(brPos);
        robo.frontRight.setTargetPosition(frPos);
        robo.powerDrive(power);
    }

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

        } else {

            pRot = -0.6 * rotMultiplier*(gamepad1.right_trigger-gamepad1.left_trigger);
            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                pRot = -rotMultiplier*(gamepad1.right_trigger-gamepad1.left_trigger);
            }
            //double gyroAngle = getHeading(); //In radiants, proper rotation, yay!!11!!
            double gyroAngle = 0;
            double magnitudeMultiplier = 0;

            // if(!toggleMap1.left_bumper){ //Removes gyroAngle from the equation meaning the robot drives normally
            //     gyroAngle = 0;
            // }
            double modifiedTheta = theta + Math.PI / 4 - gyroAngle;

            double thetaInFirstQuad = Math.abs(Math.atan(stick_y / stick_x)); //square to circle conversion
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
    public void reset(){
        if(gamepad1.b){
            flPos = 0;
            blPos = 0;
            brPos = 0;
            frPos = 0;
            robo.resetDrive();
            if(toggleMap1.y){
                robo.runWithoutEncoderDrive();
            }
            else{
                robo.runToPosDrive();
            }
        }
    }
    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    //[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[

    public void updateKeys(){
        if(gamepad2.left_bumper && cdCheck(useMap2.left_bumper, 200)){
            toggleMap2.left_bumper = toggle(toggleMap2.left_bumper);
            useMap2.left_bumper = runtime.milliseconds();
        }
        if(gamepad2.right_bumper && cdCheck(useMap2.right_bumper, 200)){
            toggleMap2.right_bumper = toggle(toggleMap2.right_bumper);
            useMap2.right_bumper = runtime.milliseconds();
        }
        if(gamepad1.y && cdCheck(useMap1.y, 200)){
            toggleMap1.y = toggle(toggleMap1.y);
            useMap1.y = runtime.milliseconds();
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



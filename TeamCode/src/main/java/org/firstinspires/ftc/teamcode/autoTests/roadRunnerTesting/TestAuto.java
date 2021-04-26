package org.firstinspires.ftc.teamcode.autoTests.roadRunnerTesting;


//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.robot.Robot;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Ladle;

@TeleOp(name = "TestAuto", group = "test")
@Disabled
public class TestAuto extends LinearOpMode{


     Ladle robo = new Ladle();

        // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public enum dir {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

//    test different values and see how to find strafe constant so strafeEncoder = nomalEncoder * constant;
    public final double strafeConstant = 1.6;

    @Override
    public void runOpMode() {
        robo.init(hardwareMap);
        robo.imu();

        robo.resetDrive();
        robo.frontLeft.setTargetPosition(0);
        robo.backLeft.setTargetPosition(0);
        robo.backRight.setTargetPosition(0);
        robo.frontRight.setTargetPosition(0);
        robo.runToPosDrive();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            int firstPosition = convertToEncoder(1000);
            int currentPosition = avgPosition();

            while(checkMovement(firstPosition, currentPosition)) {
                driveTo(firstPosition, dir.FORWARD);
                currentPosition = avgPosition();
                telemetry.addData("Current Encoder Position:", currentPosition);
            }
            telemetry.addData("Current Encoder Position:", currentPosition);

            telemetry.update();
        }

    }

    public void driveTo(int desired, dir val) {
        switch (val) {
            case FORWARD:
//                move forward distance
//                robo.runToPosDrive();
                robo.frontLeft.setTargetPosition(desired);
                robo.backLeft.setTargetPosition(desired);
                robo.backRight.setTargetPosition(desired);
                robo.frontRight.setTargetPosition(desired);
                robo.powerDrive(.3);
                break;
            case BACKWARD:
//                move backward
//                robo.runToPosDrive();
                robo.frontLeft.setTargetPosition(desired);
                robo.backLeft.setTargetPosition(desired);
                robo.backRight.setTargetPosition(desired);
                robo.frontRight.setTargetPosition(desired);
                robo.powerDrive(-.3);
                break;
            case LEFT:
                robo.frontLeft.setTargetPosition(-desired);
                robo.backLeft.setTargetPosition(desired);
                robo.backRight.setTargetPosition(-desired);
                robo.frontRight.setTargetPosition(desired);
                robo.powerDrive(.3);
                break;
//                move left
            case RIGHT:
//                move right

//                robo.runToPosDrive();
                robo.frontLeft.setTargetPosition(desired);
                robo.backLeft.setTargetPosition(-desired);
                robo.backRight.setTargetPosition(desired);
                robo.frontRight.setTargetPosition(-desired);
                robo.powerDrive(.3);
                break;
        }

    }

//    public int getDesired(double distance) {
//        return convertToEncoder(distance);
//    }

    public int avgPosition() {
        return (int) ((robo.frontLeft.getCurrentPosition() + robo.frontRight.getCurrentPosition() + robo.backLeft.getCurrentPosition() + robo.backRight.getCurrentPosition())/4);
    }


    public boolean checkMovement(int desired, int currentPosition) {
        if (Math.abs(desired - currentPosition) < (int) (.05 * desired)) {
            return (true);
        } else {
            return (false);
        }
    }
    public int convertToEncoder(double desiredDistance) {
//        gobilda ticks per revolution = 537.6
//        gobilda radius = 50 mm      diameter = 100 mm
        final double radius = 50;
        final double ticks = 537.6;
        final double distancePerRevolution = radius * 2 * Math.PI;

        final double revolutionsNeeded = desiredDistance / distancePerRevolution;

        return (int) (ticks * revolutionsNeeded);
    }
}

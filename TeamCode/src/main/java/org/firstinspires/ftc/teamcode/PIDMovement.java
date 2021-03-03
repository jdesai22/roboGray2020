package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "moving with pid", group = "test")
public class PIDMovement extends LinearOpMode {

    Ladle robo = new Ladle();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public enum dir {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    //    //while True:
////    current_time = get_current_time()
////    current_error = desire_position-current_position
////
////            p = k_p * current_error
////
////    i += k_i * (current_error * (current_time - previous_time))
////
////            if i > max_i:
////    i = max_i
////    elif i < -max_i:
////    i = -max_i
////
////            D = k_d * (current_error - previous_error) / (current_time - previous_time)
////
////    output = p + i + d
////
////            previous_error = current_error
////    previous_time = current_time

    double currentTime, currentError;
    double p, i, j;
    final double kp = .1;
    final double kd = .01;
    final double ki = 0;
    final double maxi = 1;

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


//    //while True:
////    current_time = get_current_time()
////    current_error = desire_position-current_position
////
////            p = k_p * current_error
////
////    i += k_i * (current_error * (current_time - previous_time))
////
////            if i > max_i:
////    i = max_i
////    elif i < -max_i:
////    i = -max_i
////
////            D = k_d * (current_error - previous_error) / (current_time - previous_time)
////
////    output = p + i + d
////
////            previous_error = current_error
////    previous_time = current_time

    public void pidLoop(int desired, double previousTime) {
        currentTime = runtime.milliseconds();
        currentError = desired - avgPosition();

        p = kp * currentError;

        i += ki * (currentError * (currentTime - previousTime));

        if (i > maxi) {
            i = maxi;
        } else if (i <  -maxi) {
            i = -maxi;
        }



    }

    public double getTime() {
        return currentTime;
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

    public int avgPosition() {
        return (int) ((robo.frontLeft.getCurrentPosition() + robo.frontRight.getCurrentPosition() + robo.backLeft.getCurrentPosition() + robo.backRight.getCurrentPosition())/4);
    }


    public boolean checkMovement(int desired, int currentPosition) {
        return Math.abs(desired - currentPosition) < (int) (.01 * desired);
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

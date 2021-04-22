package org.firstinspires.ftc.teamcode.hardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Arm Test", group="main")
public class ArmTest extends LinearOpMode {
    private DcMotor intake;
    private Servo servo = null;
    private ElapsedTime runtime = new ElapsedTime();
    double currentTime = 0;
    double newTime = 0;

    @Override
    public void runOpMode() {
        intake = hardwareMap.dcMotor.get("arm");
        servo = hardwareMap.servo.get("claw");


        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            intake();
            servoToLimits();
            servoAdjust();

            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }

    public void intake() {
//        if (Math.abs(gamepad1.right_stick_y) > 0.1) {
//            intake.setPower(0.3 * gamepad1.right_stick_y);
//        } else {
//            intake.setPower(0);
//        }

        intake.setPower(0.4 * gamepad1.right_stick_y);

//        if (gamepad1.a) {
////            currentTime = runtime.milliseconds();
////            intake.setTargetPosition(300);
////            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////            intake.setPower(0.5);
////            newTime = runtime.milliseconds();
////
////            if (Math.abs(intake.getCurrentPosition() - 300) < 10) {
////                intake.setPower(0);
////            }
////            if (newTime-currentTime > 20000) {
////                intake.setPower(0);
////
////            }
//        } else if (gamepad1.b) {
//            intake.setPower(0);
//        } else if(gamepad1.left_bumper){
////            intake.setTargetPosition(-300);
////            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////            intake.setPower(-.5);
////
////            if (Math.abs(intake.getCurrentPosition() + 300) < 10) {
////                intake.setPower(0);
////            }
////            if (newTime-currentTime > 20000) {
////                intake.setPower(0);
////
////            }
//        }
    }


    public void servoToLimits(){
        if(gamepad1.x){
            servo.setPosition(.5);
            // closed
        }
        else if(gamepad1.y){
            servo.setPosition(0);
            // open
        }
    }

    public void servoAdjust(){
        servo.setPosition(servo.getPosition() + (-gamepad1.left_stick_y + gamepad1.left_stick_x)/1000);
    }
}

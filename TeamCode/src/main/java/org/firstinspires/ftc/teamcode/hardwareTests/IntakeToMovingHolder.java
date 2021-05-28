package org.firstinspires.ftc.teamcode.hardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="IntakeToMovingHolder", group="Test")
public class IntakeToMovingHolder extends LinearOpMode{
    private DcMotor intake1;
    private DcMotor intake2;

    private Servo servo = null;
    private Servo servo1 = null;
    // Robot robot = new Robot();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        intake1 = hardwareMap.dcMotor.get("intake1");

//        mess w/FORWARD anD REVERSE to get it moving in right direction
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2 = hardwareMap.dcMotor.get("intake2");

//        mess w/FORWARD anD REVERSE to get it moving in right direction
        intake2.setDirection(DcMotor.Direction.FORWARD);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        servo = hardwareMap.servo.get("rightServo");
        // right servo = 5
        // left servo = 4
        servo1 = hardwareMap.servo.get("leftServo");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            intake();

            servoToLimits();
            telemetry.addData("Servo Pos1", servo.getPosition());
            telemetry.addData("Servo Pos2", servo1.getPosition());
            telemetry.update();
        }
    }

    public void intake() {
        if (gamepad1.a) {
            intake1.setPower(1);
        } else if (gamepad1.b) {
            intake1.setPower(0);
        } else if (gamepad1.x) {
            intake1.setPower(-1);
        }

        if (gamepad1.dpad_up) {
            intake2.setPower(1);
        } else if (gamepad1.dpad_down) {
            intake2.setPower(0);
        } else if (gamepad1.dpad_right) {
            intake2.setPower(-1);
        }
    }

    public void servoToLimits(){
        if(gamepad1.left_bumper){
            servo.setPosition(.269);
            servo1.setPosition(.88);
        }
        else if(gamepad1.right_bumper){
            servo.setPosition(0);
            servo1.setPosition(1);
        }
    }

}
package org.firstinspires.ftc.teamcode.hardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="PlatformTest", group="Test")
public class PlatformTest extends LinearOpMode {
    private Servo servo = null;
    private Servo servo1 = null;

    @Override
    public void runOpMode() {
        servo = hardwareMap.servo.get("rightServo");
        // right servo = 5
        // left servo = 4
        servo1 = hardwareMap.servo.get("leftServo");

        while(!opModeIsActive()){

        }

        servo.setPosition(0.05);
        servo1.setPosition(.95);

        waitForStart();
        while(opModeIsActive()){
            servoAdjust1();
            servoAdjust2();
            servoToLimits();
            telemetry.addData("Servo Pos1", servo.getPosition());
            telemetry.addData("Servo Pos2", servo1.getPosition());

            telemetry.update();
        }
    }

    public void servoAdjust1(){
        servo.setPosition(servo.getPosition() + (-gamepad1.left_stick_y )/5);
    }

    public void servoAdjust2(){
        servo.setPosition(servo.getPosition() + (-gamepad1.right_stick_y)/5);
    }

    public void servoToLimits(){
        if(gamepad1.b){
            servo.setPosition(.42);
            servo1.setPosition(.58);
        }
        else if(gamepad1.a){
            servo.setPosition(0.08);
            servo1.setPosition(.92);
        }
    }
}
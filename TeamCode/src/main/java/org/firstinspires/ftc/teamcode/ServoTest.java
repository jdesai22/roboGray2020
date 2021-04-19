package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="ServoTest", group="Test")
public class ServoTest extends LinearOpMode {
    private Servo servo = null;
    @Override
    public void runOpMode() {
        servo = hardwareMap.servo.get("claw");


        while(!opModeIsActive()){

        }

        while(opModeIsActive()){
            servoAdjust();
            servoToLimits();
            telemetry.addData("Servo Pos", servo.getPosition());
            telemetry.update();
        }
    }

    public void servoAdjust(){
        servo.setPosition(servo.getPosition() + (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_y + gamepad1.right_stick_x)/1000);
    }
    public void servoToLimits(){
        if(gamepad1.b){
            servo.setPosition(1);
        }
        else if(gamepad1.a){
            servo.setPosition(0);
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="LaunchTest", group="Test")
public class LaunchTest extends LinearOpMode {
    private Servo pusher = null;
//    private GoBILDA5202Series launcher = null;

    private DcMotor launcher = null;

    @Override
    public void runOpMode() {

        pusher = hardwareMap.servo.get("pusher");
        launcher = hardwareMap.dcMotor.get("launcher");

        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()){
            moveServo();
            setServo();
            launch();
            telemetry.addData("Servo Pos", pusher.getPosition());
            telemetry.update();
        }
    }

    public void moveServo(){
        pusher.setPosition(pusher.getPosition() + (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_y + gamepad1.right_stick_x)/1000);
    }
    public void setServo(){
        if(gamepad1.left_bumper){
            pusher.setPosition(1);
        }
        else if(gamepad1.right_bumper){
            pusher.setPosition(0);
        }
    }

    public void launch() {
        if (gamepad1.a) {
            launcher.setPower(1);
        } else if (gamepad1.b) {
            launcher.setPower(0);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="LaunchTest", group="Test")
public class LaunchTest extends LinearOpMode {
    private Servo pusher = null;
//    private GoBILDA5202Series launcher = null;

    private DcMotor launcher = null;

    ElapsedTime launchTime = new ElapsedTime();


    //    VARIABLES
    double launchBuffer = 0.5;
    boolean launcherRunning = false;

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
//        if(gamepad1.left_bumper){
//            pusher.setPosition(1);
//        }
//        else if(gamepad1.right_bumper){
//            pusher.setPosition(0);
//        }
        if (launchTime.seconds() - launchTime.startTime() > launchBuffer && gamepad1.right_trigger > 0.1 && launcherRunning && pusher.getPosition() == 0.5) {
            pusher.setPosition(0.75);
            launchTime.reset();
        } else if (launchTime.seconds() - launchTime.startTime() > launchBuffer && pusher.getPosition() == 0.75){
            pusher.setPosition(0.5);
        }
    }

    public void launch() {
//        if (gamepad1.a) {
//            launcher.setPower(1);
//        } else if (gamepad1.b) {
//            launcher.setPower(0);
//        }
        if(gamepad1.a) {
            launcher.setPower(1);
            launcherRunning = true;
        } else if (gamepad1.x){
            launcher.setPower(0);
            launcherRunning = false;
        }
    }

}

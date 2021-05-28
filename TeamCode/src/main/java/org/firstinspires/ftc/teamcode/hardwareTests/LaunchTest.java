package org.firstinspires.ftc.teamcode.hardwareTests;

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
    boolean launcherRunning = false;
    boolean servoMoving = false;

//    tune these constants
    double launchBuffer = 0.5;
    final double upServo = .48;// originally .52
    final double bottomServo = .83;

    @Override
    public void runOpMode() {

        pusher = hardwareMap.servo.get("pusher");
        launcher = hardwareMap.dcMotor.get("launcher");

        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pusher.setPosition(bottomServo);

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
        if (gamepad1.right_bumper && launchTime.milliseconds() >= 300 && !servoMoving) {
            pusher.setPosition(upServo);
            servoMoving = true;
            launchTime.reset();
        }

        if (launchTime.milliseconds() >= 300 && servoMoving) {
            pusher.setPosition(bottomServo);
            servoMoving = false;
            launchTime.reset();
        }
    }


    public void launch() {
//        if (gamepad1.a) {
//            launcher.setPower(1);
//        } else if (gamepad1.b) {
//            launcher.setPower(0);
//        }
        if(gamepad1.a) {
            launcher.setPower(.6);
        } else if (gamepad1.x){
            launcher.setPower(0);
        }
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
        servo = hardwareMap.servo.get("grasp");


        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            intake();
            motorPower();
            servoAdjust();
            servoToLimits();

            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }

    public void intake() {
        if (gamepad1.a) {
            currentTime = runtime.milliseconds();
            intake.setPower(0.25);
            newTime = runtime.milliseconds();

            if (newTime-currentTime > 500) {
                intake.setPower(0);

            }
        } else if (gamepad1.b) {
            intake.setPower(0);
        }
    }

    public void motorPower(){
        intake.setPower(gamepad1.left_stick_y);
        intake.setPower(gamepad1.right_stick_y);
    }

    public void servoAdjust(){
        servo.setPosition(servo.getPosition() + (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_y + gamepad1.right_stick_x)/1000);
    }
    public void servoToLimits(){
        if(gamepad1.x){
            servo.setPosition(.5);
        }
        else if(gamepad1.y){
            servo.setPosition(0);
        }
    }
}

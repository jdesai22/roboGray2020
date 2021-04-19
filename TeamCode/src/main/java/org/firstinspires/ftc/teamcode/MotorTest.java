package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="newMotorTest", group="Test")

public class MotorTest extends LinearOpMode{
    private DcMotor intake;

    // Robot robot = new Robot();

        // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        intake = hardwareMap.dcMotor.get("arm");

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
            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }

    public void intake() {
        if (gamepad1.a) {
            intake.setPower(0.25);
        } else if (gamepad1.b) {
            intake.setPower(0);
        }
    }

    public void motorPower(){
        intake.setPower(gamepad1.left_stick_y);
        intake.setPower(gamepad1.right_stick_y);
    }

}
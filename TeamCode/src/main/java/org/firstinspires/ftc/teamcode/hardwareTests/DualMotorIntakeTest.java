package org.firstinspires.ftc.teamcode.hardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="dualMotorTest", group="Test")
@Disabled
public class DualMotorIntakeTest extends LinearOpMode{
    private DcMotor intake1;
    private DcMotor intake2;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            intake();
            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }

    public void intake() {
        if (gamepad1.a) {
            intake1.setPower(1);
        } else if (gamepad1.b) {
            intake1.setPower(0);
        }

        if (gamepad1.x) {
            intake2.setPower(1);
        } else if (gamepad1.y) {
            intake2.setPower(0);
        }
    }

}
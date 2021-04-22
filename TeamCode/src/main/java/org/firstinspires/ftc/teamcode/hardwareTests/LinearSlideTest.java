package org.firstinspires.ftc.teamcode.hardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class LinearSlideTest extends LinearOpMode {
    public DcMotor slides = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        slides = hardwareMap.dcMotor.get("slides");

        slides.setDirection(DcMotor.Direction.FORWARD);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            moveSlides();
            telemetry.update();
        }
    }

    public void moveSlides() {
        slides.setPower(0.4 * gamepad1.right_stick_y);
        telemetry.addData("Slide Position: ", slides.getCurrentPosition());
    }
}

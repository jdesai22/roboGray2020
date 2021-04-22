package org.firstinspires.ftc.teamcode.autoTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Ladle;

@TeleOp
public class ShootThreeRings extends LinearOpMode {

    Ladle robo = new Ladle();

    final double upServo = .50;
    final double bottomServo = .83;

    boolean servoMoving = false;

    public ElapsedTime launchTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robo.init(hardwareMap);
        robo.imu();

        waitForStart();

        if (isStopRequested()) return;

        launchTime.reset();

        robo.launcher.setPower(1);

        sleep(300);

        for (int i = 0; i <= 5; i++) {
            boolean cycle = false;

            while (!cycle) {
                if (launchTime.milliseconds() >= 350 && !servoMoving) {
                    robo.pusher.setPosition(upServo);
                    servoMoving = true;
                    launchTime.reset();
                }
                if (launchTime.milliseconds() >= 350 && servoMoving) {
                    robo.pusher.setPosition(bottomServo);
                    servoMoving = false;
                    launchTime.reset();
                    cycle = true;
                }
            }
        }

        robo.launcher.setPower(0);
    }
}

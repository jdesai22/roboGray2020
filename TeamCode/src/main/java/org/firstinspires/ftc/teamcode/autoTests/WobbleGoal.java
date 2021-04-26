package org.firstinspires.ftc.teamcode.autoTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Ladle;


@TeleOp
public class WobbleGoal extends LinearOpMode {

    Ladle robo = new Ladle();
    

    @Override
    public void runOpMode() {
        robo.autoInit(hardwareMap);
        robo.grasp.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.x) {
                dropWobble(robo);
            } else if(gamepad1.y) {
                raiseWobble(robo);
            }
        }




    }

    public void raiseWobble(Ladle robo) {
        robo.grasp.setPosition(0);

        sleep(500);
        //neg power = drop
        robo.arm.setPower(-.6);
        sleep(700);
        robo.arm.setPower(0);




    }

    public void dropWobble(Ladle robo) {

        robo.arm.setPower(.4);
        sleep(1000);


        robo.arm.setPower(0);
        robo.grasp.setPosition(0.5);



    }

}

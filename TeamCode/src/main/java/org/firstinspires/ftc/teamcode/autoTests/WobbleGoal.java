package org.firstinspires.ftc.teamcode.autoTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Ladle;

@Disabled
@TeleOp
public class WobbleGoal extends LinearOpMode {

    Ladle robo = new Ladle();
    

    @Override
    public void runOpMode() {
        robo.init(hardwareMap);
        robo.imu();

        waitForStart();

        if (isStopRequested()) return;



    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunnerTuner.drive.SampleMecanumDrive;

import java.util.ArrayList;


@Autonomous(name = "rr auto", group = "Important")
public class Auto extends LinearOpMode {

    Ladle robo = new Ladle();

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(10, -5, 0.0);
        drive.setPoseEstimate(startPose);

        robo.init(hardwareMap);
        robo.imu();

        waitForStart();

        while(!opModeIsActive()) {

            telemetry.update();
        }

        while(opModeIsActive()){



            telemetry.addData("Back Left", robo.backLeft.getCurrentPosition());
            telemetry.addData("Front Left", robo.frontLeft.getCurrentPosition());
            telemetry.addData("Back Right", robo.backRight.getCurrentPosition());
            telemetry.addData("Front Right", robo.frontRight.getCurrentPosition());

            telemetry.update();
        }

    }


    private ArrayList<Trajectory> TrajGen(SampleMecanumDrive drive, Pose2d startPose, int ringNum) {

        ArrayList<Trajectory> movement = new ArrayList<Trajectory>();

        return (movement);
    }
}

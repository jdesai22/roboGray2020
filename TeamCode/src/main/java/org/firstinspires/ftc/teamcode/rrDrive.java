package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunnerTuner.drive.SampleMecanumDrive;


@TeleOp(group = "drive")
@Disabled
public class rrDrive extends LinearOpMode {

    ToggleMap toggleMap1 = new ToggleMap();
    UseMap useMap1 = new UseMap();
    Ladle robo = new Ladle();
    SampleMecanumDrive drive;


    private ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime launchTime = new ElapsedTime();

    final double upServo = .48;
    final double bottomServo = .83;

    boolean servoMoving = false;

    @Override
    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robo.autoInit(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);

//        Mode currentState = Mode.strafe1;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {

            updateKeys();

            if (gamepad1.y) {
                telemetry.addData("status", "powershots");
                powershots();

            }  else if (gamepad1.dpad_right) {

                setPoseRightWall();

            } else if (gamepad1.dpad_up) {

                autoAim();

            } else if (gamepad1.dpad_down) {

                setShootPose();
            }

            else {

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                //see if this last heading/rotation works
                                -gamepad1.right_stick_x
                        )
                );

                drive.update();

            }


//            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

//            if (toggleMap1.y) {
//                robo.intake1.setPower(1);
//            }
        }
    }

    public void powershots() {
        robo.launcher.setPower(.75);

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // is this needed?
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        test this
        drive.setPoseEstimate(new Pose2d(-0.5, 5.0));

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(14.5)
                .addDisplacementMarker(18, () -> {
                    shootRings(robo, 2);
                    telemetry.addData("shoot ring", "1");
                })
//                .addTemporalMarker(1, ()->{})
                .build();


        drive.followTrajectory(traj1);

        sleep(400);

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(8)
                .addDisplacementMarker(() -> {
                    shootRings(robo, 1);

                })
                .build();

        drive.followTrajectory(traj2);

//        shootRings(robo, 1);

        sleep(400);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(7)
                .addDisplacementMarker(6, () -> {
                    shootRings(robo, 1);
                })
                .build();

        drive.followTrajectory(traj3);

//        shootRings(robo, 1);
//        sleep(400);

        robo.launcher.setPower(0);
    }

    public void autoAim() {
        Pose2d pose = drive.getPoseEstimate();
        Trajectory lineUp = drive.trajectoryBuilder(pose)
                .lineToSplineHeading(new Pose2d(-.5, -34, Math.toRadians(0.0)))
                .build();

        drive.followTrajectory(lineUp);
    }

    public void setPoseRightWall() {
        //fine tune
        drive.setPoseEstimate(new Pose2d(-0.5, -66));
    }

    public void setShootPose() {
        drive.setPoseEstimate(new Pose2d(-0.5, -34));
    }
//    public Pose2d getPose() {
//        return drive.getPoseEstimate();
//    }

    public void shootRings(Ladle robo, double shots) {

        launchTime.reset();

//        sleep(250);

        for (int i = 0; i < shots; i++) {
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
    }

    public void updateKeys(){

        if(gamepad1.y && cdCheck(useMap1.y, 500)){
            toggleMap1.y = toggle(toggleMap1.y);
            useMap1.y = runtime.milliseconds();
        }

    }

    public boolean cdCheck(double key, int cdTime){
        return runtime.milliseconds() - key > cdTime;
    }
    
    public boolean toggle(boolean variable){
        if(variable == true){
            variable = false;
        }
        else if(variable == false){
            variable = true;
        }
        return variable;

    }
}

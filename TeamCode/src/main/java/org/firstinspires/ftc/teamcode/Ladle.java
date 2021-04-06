package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


public class Ladle {
    //////////////////
    /* DECLARATIONS */
    //////////////////

    //DRIVE//
    public DcMotor frontLeft   = null;
    public DcMotor backLeft    = null;
    public DcMotor backRight   = null;
    public DcMotor frontRight  = null;

    //INTAKE//
    public DcMotor intake1      = null;
    public DcMotor intake2      = null;

    //LAUNCHER//
    public DcMotor launcher    = null;

    //PUSHER//
    public Servo pusher      = null;

    //ARM//
    public DcMotor arm = null;
    public Servo grasp = null;

//    tune these
    final int  upArm = 500;
    final int downArm = 100;
    final double closeGrasp = .8;
    final double openGrasp = .2;

    //IMU//
    BNO055IMU imu;

    HardwareMap hwMap           =  null;

    /* Constructor */
    public void ladle (){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        //////////////////////////////////
        /* RETRIEVING STUFF FROM PHONES */
        //////////////////////////////////

        //DRIVE//
        frontLeft   = hwMap.dcMotor.get("fl");
        backLeft    = hwMap.dcMotor.get("bl");
        backRight   = hwMap.dcMotor.get("br");
        frontRight  = hwMap.dcMotor.get("fr");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //INTAKE// check forward and reverse
        intake1 = hwMap.dcMotor.get("intake1");
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2 = hwMap.dcMotor.get("intake2");
        intake2.setDirection(DcMotor.Direction.FORWARD);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        //Launcher//
        launcher = hwMap.dcMotor.get("launcher");
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //PUSHER//
//        pusher = hwMap.servo.get("pusher");

        //ARM//
//        arm = hwMap.dcMotor.get("arm");
//        arm.setDirection(DcMotor.Direction.FORWARD); //adjust accordingly
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        grasp = hwMap.servo.get("grasp");


        //IMU//
        imu = hwMap.get(BNO055IMU.class, "imu");


    }

    public void clearArmVoltage() {
        if(!arm.isBusy()) {
            arm.setPower(0);
        }
    }

    public void raiseArm() {
        arm.setTargetPosition(upArm);
        arm.setPower(.6);
    }

    public void lowerArm() {
        arm.setTargetPosition(downArm);
        arm.setPower(-.6);
    }

    public void graspWobble() {
        grasp.setPosition(closeGrasp);
    }

    public void ungraspWobble() {
        grasp.setPosition(openGrasp);
    }

    public void acquireWobbleGoal() {
        graspWobble();

        if(Math.abs(grasp.getPosition()-closeGrasp) < 0.05) {
            raiseArm();
        }
    }

    public void depositWobbleGoal() {
        lowerArm();

        if (Math.abs(arm.getCurrentPosition()-downArm) <  10) {
            ungraspWobble();
        }
    }

    public void mecanumDrive(double pX, double pY, double pRot){
        frontLeft.setPower(pY + pRot);
        backLeft.setPower(pX - pRot);
        backRight.setPower(pY - pRot);
        frontRight.setPower(pX + pRot);
    }


    public void powerDrive(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(power);
    }


    public void resetDrive(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void runToPosDrive(){
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void runWithoutEncoderDrive(){
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void imu(){
        /* IMU STUFF */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
    }
}

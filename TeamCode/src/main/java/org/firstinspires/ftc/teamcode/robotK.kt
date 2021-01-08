package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap


class robotK  /* Constructor */ {
    //////////////////
    /* DECLARATIONS */ //////////////////
    //DRIVE//
    @JvmField
    var frontLeft: DcMotor? = null
    @JvmField
    var backLeft: DcMotor? = null
    @JvmField
    var backRight: DcMotor? = null
    @JvmField
    var frontRight: DcMotor? = null

    //IMU//
    var imu: BNO055IMU? = null
    var hwMap: HardwareMap? = null

    /* Initialize standard Hardware interfaces */
    fun init(hwMap: HardwareMap) {
        //////////////////////////////////
        /* RETRIEVING STUFF FROM PHONES */
        //////////////////////////////////

        //DRIVE//
        frontLeft = hwMap.dcMotor["fl"]
        backLeft = hwMap.dcMotor["bl"]
        backRight = hwMap.dcMotor["br"]
        frontRight = hwMap.dcMotor["fr"]

        // FIGURE OUT IF WE WANT DOUBLE BANG !! OR QUESTION MARK?

        frontLeft!!.setDirection(DcMotor.Direction.REVERSE)   //SHOULD BE WORKING
        backLeft!!.setDirection(DcMotor.Direction.FORWARD)
        backRight!!.setDirection(DcMotor.Direction.FORWARD)
        frontRight!!.setDirection(DcMotor.Direction.REVERSE)

        frontLeft!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        backLeft!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        backRight!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        frontRight!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)


        //IMU//
        imu = hwMap.get(BNO055IMU::class.java, "imu")
    }

    fun mecanumDrive(pX: Double, pY: Double, pRot: Double) {
        frontLeft!!.power = pY + pRot
        backLeft!!.power = pX - pRot
        backRight!!.power = pY - pRot
        frontRight!!.power = pX + pRot
    }

    fun powerDrive(power: Double) {
        frontLeft!!.power = power
        backLeft!!.power = power
        backRight!!.power = power
        frontRight!!.power = power
    }

    fun resetDrive() {
        frontLeft!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backLeft!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backRight!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        frontRight!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun runToPosDrive() {
        frontLeft!!.mode = DcMotor.RunMode.RUN_TO_POSITION
        backLeft!!.mode = DcMotor.RunMode.RUN_TO_POSITION
        backRight!!.mode = DcMotor.RunMode.RUN_TO_POSITION
        frontRight!!.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun runWithoutEncoderDrive() {
        frontLeft!!.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backLeft!!.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backRight!!.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        frontRight!!.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun imu() {
        /* IMU STUFF */
        val parameters = BNO055IMU.Parameters()
        parameters.mode = BNO055IMU.SensorMode.IMU
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.loggingEnabled = false
        imu!!.initialize(parameters)
    }
}
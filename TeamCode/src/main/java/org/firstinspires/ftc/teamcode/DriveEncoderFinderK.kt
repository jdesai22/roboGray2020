package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime

@TeleOp(name = "DriveEncoderFinder", group = "main")
class DriveEncoderFinderK : LinearOpMode() {
    var robo = robotK()
    var toggleMap1 = toggleMapK()
    var useMap1 = useMapK()
    var toggleMap2 = toggleMapK()
    var useMap2 = useMapK()

    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    /* V * A * R * I * A * B * E * S */ ///* V * A * R * I * A * B * E * S */
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //int chungoidPos = 0;
    private val runtime = ElapsedTime()
    override fun runOpMode() {
        robo.init(hardwareMap)
        robo.imu()
        robo.runWithoutEncoderDrive()
        var startTime = 0.0
        while (!opModeIsActive()) {
            telemetry.update()
        }
        robo.resetDrive()
        if (toggleMap1.y) {
            robo.runWithoutEncoderDrive()
        } else {
            robo.runToPosDrive()
        }
        startTime = runtime.milliseconds()
        while (opModeIsActive()) {
            updateKeys()
            //Player 1
            if (toggleMap1.y) {
                telemetry.addData("Drive", "Manual")
                drive()
            } else {
                telemetry.addData("Drive", "Encoder")
                encoderDrive(0.5)
            }
            reset()
            telemetry.addData("fl", robo.frontLeft.currentPosition)
            telemetry.addData("bl", robo.backLeft.currentPosition)
            telemetry.addData("br", robo.backRight.currentPosition)
            telemetry.addData("fr", robo.frontRight.currentPosition)
            telemetry.update()
        }
    }

    //Player 1
    var flPos = 0
    var blPos = 0
    var brPos = 0
    var frPos = 0
    fun encoderDrive(power: Double) {
        val stick_x = -gamepad1.left_stick_x.toDouble()
        val stick_y = gamepad1.left_stick_y.toDouble()
        var pX = 0.0
        var pY = 0.0
        var pRot = 0.0
        val rotMultiplier = 0.6
        val theta = Math.atan2(stick_y, stick_x) //Arctan2 doesn't have bad range restriction
        if (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down) {
            val mag = 0.25
            if (gamepad1.dpad_up) {
                pX = mag
                pY = -mag
            } else if (gamepad1.dpad_left) {
                pX = 2 * mag
                pY = 2 * mag
            } else if (gamepad1.dpad_down) {
                pX = -mag
                pY = mag
            } else if (gamepad1.dpad_right) {
                pX = -2 * mag
                pY = -2 * mag
            }
            pRot = -rotMultiplier * (gamepad1.right_trigger - gamepad1.left_trigger)
            robo.mecanumDrive(pX, pY, pRot)
        } else {
            pRot = -0.6 * rotMultiplier * (gamepad1.right_trigger - gamepad1.left_trigger)
            if (gamepad1.left_stick_y == 0f && gamepad1.left_stick_x == 0f) {
                pRot = -rotMultiplier * (gamepad1.right_trigger - gamepad1.left_trigger)
            }
            //double gyroAngle = getHeading(); //In radiants, proper rotation, yay!!11!!
            val gyroAngle = 0.0
            var magnitudeMultiplier = 0.0

            // if(!toggleMap1.left_bumper){ //Removes gyroAngle from the equation meaning the robot drives normally
            //     gyroAngle = 0;
            // }
            val modifiedTheta = theta + Math.PI / 4 - gyroAngle
            val thetaInFirstQuad = Math.abs(Math.atan(stick_y / stick_x)) //square to circle conversion
            if (thetaInFirstQuad > Math.PI / 4) {
                magnitudeMultiplier = Math.sin(thetaInFirstQuad) //Works because we know y is 1 when theta > Math.pi/4
            } else if (thetaInFirstQuad <= Math.PI / 4) {
                magnitudeMultiplier = Math.cos(thetaInFirstQuad) //Works because we know x is 1 when theta < Math.pi/4
            }
            val magnitude = Math.sqrt(Math.pow(stick_x, 2.0) + Math.pow(stick_y, 2.0)) * magnitudeMultiplier * (1 - Math.abs(pRot)) //Multiplied by (1-pRot) so it doesn't go over 1 with rotating
            pX = magnitude * Math.cos(modifiedTheta)
            pY = magnitude * Math.sin(modifiedTheta)
        }
        val bleh = 50
        flPos += bleh * (pY + pRot).toInt()
        blPos += bleh * (pX - pRot).toInt()
        brPos += bleh * (pY - pRot).toInt()
        frPos += bleh * (pX + pRot).toInt()
        robo.frontLeft.targetPosition= flPos // test this -> robot.frontleft.targetPosition: int? = flpos
        robo.backLeft.targetPosition = blPos
        robo.backRight.targetPosition = brPos
        robo.frontRight.targetPosition = frPos
        robo.powerDrive(power)


    }

    fun drive() {
        val stick_x = -gamepad1.left_stick_x.toDouble()
        val stick_y = gamepad1.left_stick_y.toDouble()
        var pX = 0.0
        var pY = 0.0
        var pRot = 0.0
        val rotMultiplier = 0.6
        val theta = Math.atan2(stick_y, stick_x) //Arctan2 doesn't have bad range restriction
        if (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down) {
            val mag = 0.25
            if (gamepad1.dpad_up) {
                pX = mag
                pY = -mag
            } else if (gamepad1.dpad_left) {
                pX = 2 * mag
                pY = 2 * mag
            } else if (gamepad1.dpad_down) {
                pX = -mag
                pY = mag
            } else if (gamepad1.dpad_right) {
                pX = -2 * mag
                pY = -2 * mag
            }
            pRot = -rotMultiplier * (gamepad1.right_trigger - gamepad1.left_trigger)
            robo.mecanumDrive(pX, pY, pRot)
        } else {
            pRot = -0.6 * rotMultiplier * (gamepad1.right_trigger - gamepad1.left_trigger)
            if (gamepad1.left_stick_y == 0f && gamepad1.left_stick_x == 0f) {
                pRot = -rotMultiplier * (gamepad1.right_trigger - gamepad1.left_trigger)
            }
            //double gyroAngle = getHeading(); //In radiants, proper rotation, yay!!11!!
            val gyroAngle = 0.0
            var magnitudeMultiplier = 0.0

            // if(!toggleMap1.left_bumper){ //Removes gyroAngle from the equation meaning the robot drives normally
            //     gyroAngle = 0;
            // }
            val modifiedTheta = theta + Math.PI / 4 - gyroAngle
            val thetaInFirstQuad = Math.abs(Math.atan(stick_y / stick_x)) //square to circle conversion
            if (thetaInFirstQuad > Math.PI / 4) {
                magnitudeMultiplier = Math.sin(thetaInFirstQuad) //Works because we know y is 1 when theta > Math.pi/4
            } else if (thetaInFirstQuad <= Math.PI / 4) {
                magnitudeMultiplier = Math.cos(thetaInFirstQuad) //Works because we know x is 1 when theta < Math.pi/4
            }
            val magnitude = Math.sqrt(Math.pow(stick_x, 2.0) + Math.pow(stick_y, 2.0)) * magnitudeMultiplier * (1 - Math.abs(pRot)) //Multiplied by (1-pRot) so it doesn't go over 1 with rotating
            pX = magnitude * Math.cos(modifiedTheta)
            pY = magnitude * Math.sin(modifiedTheta)
            robo.mecanumDrive(pX, pY, pRot)
        }
    }

    fun reset() {
        if (gamepad1.b) {
            flPos = 0
            blPos = 0
            brPos = 0
            frPos = 0
            robo.resetDrive()
            if (toggleMap1.y) {
                robo.runWithoutEncoderDrive()
            } else {
                robo.runToPosDrive()
            }
        }
    }

    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    //[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[
    fun updateKeys() {
        if (gamepad2.left_bumper && cdCheck(useMap2.left_bumper, 200)) {
            toggleMap2.left_bumper = toggle(toggleMap2.left_bumper)
            useMap2.left_bumper = runtime.milliseconds()
        }
        if (gamepad2.right_bumper && cdCheck(useMap2.right_bumper, 200)) {
            toggleMap2.right_bumper = toggle(toggleMap2.right_bumper)
            useMap2.right_bumper = runtime.milliseconds()
        }
        if (gamepad1.y && cdCheck(useMap1.y, 200)) {
            toggleMap1.y = toggle(toggleMap1.y)
            useMap1.y = runtime.milliseconds()
        }
    }

    fun cdCheck(key: Double, cdTime: Int): Boolean {
        return runtime.milliseconds() - key > cdTime
    }

    fun toggle(variable: Boolean): Boolean {
        var variable = variable
        if (variable == true) {
            variable = false
        } else if (variable == false) {
            variable = true
        }
        return variable
    }
}
package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime

@TeleOp(name = "MainTeleOp", group = "main")
class MainTeleOpK : LinearOpMode() {
    var robo = robotK()

    //    for controller 1
    var toggleMap1 = toggleMapK()
    var useMap1 = useMapK()

    //    for controller 2
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
        //Check if slides hold position when transitioned
        var startTime = 0.0
        while (!opModeIsActive()) {
            resetEncoders()
            telemetry.update()
        }
        startTime = runtime.milliseconds()
        while (opModeIsActive()) {
            updateKeys()
            drive()
            resetEncoders()
            telemetry.update()
        }
    }

    //Player 1
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
            val gyroAngle = 0.0
            var magnitudeMultiplier = 0.0

//            ALL TRIG IS IN RADIANS
            val modifiedTheta = theta + Math.PI / 4 - gyroAngle
            val thetaInFirstQuad = Math.abs(Math.atan(stick_y / stick_x)) //square to circle conversion
            //            arctan is same as tan inverse
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

    fun resetEncoders() {
        if (gamepad2.y && gamepad2.b) {
            if (!toggleMap2.b) {
                useMap2.b = runtime.milliseconds() //Using useMap weirdly here
            }
            toggleMap2.b = true //Using toggleMap weirdly here
            if (cdCheck(useMap2.b, 2000)) {
                telemetry.addData(">", "Encoders Successfully Reset")
            } else {
                telemetry.addData("WARNING", "RESETTING ENCODERS")
            }
        } else {
            toggleMap2.b = false
        }
    }

    ////////////////////////////////
    // TOGGLES ////////// USE MAP //
    //[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[
    fun updateKeys() {
        if (gamepad1.left_bumper && cdCheck(useMap1.left_bumper, 500)) {
            toggleMap1.left_bumper = toggle(toggleMap1.left_bumper)
            useMap1.left_bumper = runtime.milliseconds()
            toggleMap1.right_bumper = false
        }
        if (gamepad1.right_bumper && cdCheck(useMap1.right_bumper, 500)) {
            toggleMap1.right_bumper = toggle(toggleMap1.right_bumper)
            useMap1.right_bumper = runtime.milliseconds()
            toggleMap1.left_bumper = false
        }
        if (gamepad1.b && cdCheck(useMap1.b, 500)) {
            toggleMap1.b = toggle(toggleMap1.b)
            useMap1.b = runtime.milliseconds()
            toggleMap1.x = false
        }
        if (gamepad2.left_bumper) {
            toggleMap1.left_bumper = false
            toggleMap1.right_bumper = false
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
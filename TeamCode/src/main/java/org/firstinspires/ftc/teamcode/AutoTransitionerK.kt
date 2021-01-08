package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl

//import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
/**
 * Created by KNO3 Robotics
 * AutoTransitioner is a utility to automatically initialize the teleop program of your choice
 * after the autonomous period ends. To use AutoTransitioner, go to your OpMode/LinearOpMode,
 * and place the following line of code in your init() method or before your waitForStart()
 * (for OpMode and LinearOpMode, respectively):
 * AutoTransitioner.transitionOnStop(this, "Robot Teleop");
 * Where 'Robot Teleop' is replaced with the NAME of your teleop program. See full documentation
 * on kno3.net/resources for more info.
 */
class AutoTransitionerK private constructor() : Thread() {
    private var onStop: OpMode? = null
    private var transitionTo: String? = null
    private var opModeManager: OpModeManagerImpl? = null
    override fun run() {
        try {
            while (true) { //Loop
                synchronized(this) { //Synchronized to prevent weird conditions
                    //If there is a transition set up and the active op mode is no longer the one
                    //the transition was set up with, proceed with the transition
                    if (onStop != null && opModeManager!!.activeOpMode !== onStop) {
                        sleep(1000) //Wait 1 second to prevent weird conditions
                        opModeManager!!.initActiveOpMode(transitionTo) //Request initialization of the teleop
                        reset() //Reset the AutoTransitioner
                    }
                }
                sleep(50) //Sleep 50 seconds to minimize performance impact to the rest of your program
            }
        } catch (ex: InterruptedException) {
            //Log.e(FtcRobotControllerActivity.TAG, "AutoTransitioner shutdown, thread interrupted");
        }
    }

    private fun setNewTransition(onStop: OpMode, transitionTo: String) {
        synchronized(this) {
            //Synchronized to prevent wierd conditions
            this.onStop = onStop
            this.transitionTo = transitionTo
            opModeManager = onStop.internalOpModeServices as OpModeManagerImpl //Store OpModeManagerImpl
        }
    }

    private fun reset() {
        onStop = null
        transitionTo = null
        opModeManager = null
    }

    companion object {
        private val INSTANCE = AutoTransitionerK() //Create singleton instance

        /**
         * Setup the next transition
         * @param onStop The program you'll be transitioning from (usually 'this')
         * @param transitionTo The name of the program you want to transition to
         */
        fun transitionOnStop(onStop: OpMode, transitionTo: String) {
            INSTANCE.setNewTransition(onStop, transitionTo)
        }
    }

    init {
        start() //Start the watcher thread
    }
}
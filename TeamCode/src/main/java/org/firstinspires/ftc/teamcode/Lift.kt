package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.FunctionalCommand
import com.amarcolini.joos.control.FeedforwardCoefficients
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.MotorGroup
import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.HardwareMap

class Lift(hMap: HardwareMap) : AbstractComponent() {
    private val ticksPerInch = 1.0
    private val group = MotorGroup(
        hMap,
        Motor.Kind.GOBILDA_30,
        "left_lift" to false, "right_lift" to false
    ).apply {
        zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE
        runMode = Motor.RunMode.RUN_TO_POSITION
        positionCoefficients = PIDCoefficients(0.0, 0.0, 0.0)
//        feedforwardCoefficients = FeedforwardCoefficients()
    }
    private val emergencySensor = hMap.get(RevTouchSensor::class.java, "emergency_toucher")

    init {
        subcomponents += group
    }

    fun init(): Command = FunctionalCommand(
        init = {
            group.setPower(-0.01)
        },
        isFinished = emergencySensor::isPressed,
        end = {
            group.setPower(0.0)
            group.resetEncoder()
        },
        isInterruptable = false
    )

    fun goToHeight(height: Double) = group.goToPosition((height * ticksPerInch).toInt())
        .stopWhen(emergencySensor::isPressed)
}
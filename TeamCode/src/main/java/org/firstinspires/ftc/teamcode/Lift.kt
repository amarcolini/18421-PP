package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandScheduler
import com.amarcolini.joos.command.FunctionalCommand
import com.amarcolini.joos.control.FeedforwardCoefficients
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.control.PIDFController
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.MotorGroup
import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.roundToInt

class Lift(hMap: HardwareMap) : AbstractComponent() {
    private val ticksPerInch = 1.0
    private val leftMotor = hMap.get(DcMotorEx::class.java, "left_lift")
    private var leftOffset: Int = 0
    private val rightMotor = hMap.get(DcMotorEx::class.java, "right_lift")
    private var rightOffset: Int = 0
    val controller: PIDFController = PIDFController(PIDCoefficients())

    init {
        leftMotor.direction = DcMotorSimple.Direction.REVERSE
        rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        leftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
    private val emergencySensor = hMap.get(RevTouchSensor::class.java, "emergency_toucher")

    companion object {
        var initialized = false
            private set
        private var leftPosition = 0
        private var rightPosition = 0
    }

    override fun update() {
        val position = 0.5 * (leftMotor.currentPosition + rightMotor.currentPosition)
        val output = controller.update(position)
        setPower(output)
        CommandScheduler.telemetry.addLine("updating motors")
    }
    
    fun setPower(power: Double) {
        leftMotor.power = power
        rightMotor.power = power
    }
    
    private fun resetEncoders() {
        leftOffset = -leftMotor.currentPosition
        rightOffset = -rightMotor.currentPosition
    }
    
    fun getPosition() = 0.5 * (leftMotor.currentPosition + leftOffset + rightMotor.currentPosition + rightOffset)

    fun init(): Command = FunctionalCommand(
        init = {
            setPower(-0.03)
        },
        isFinished = emergencySensor::isPressed,
        end = {
            setPower(0.0)
            resetEncoders()
            initialized = true
        },
        isInterruptable = false
    )

    fun goToPosition(position: Int): Command = FunctionalCommand(
        init = {
            controller.targetPosition = position.toDouble()
        },
        isFinished = {
            controller.isAtSetPoint() || emergencySensor.isPressed
        }
    )

    fun goToHeight(height: Double): Command = goToPosition((height * ticksPerInch).roundToInt())
}
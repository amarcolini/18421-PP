package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.*
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.control.PIDFController
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.profile.MotionProfileGenerator
import com.amarcolini.joos.profile.MotionState
import com.amarcolini.joos.util.epsilonEquals
import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.roundToInt

@JoosConfig
class Lift(hMap: HardwareMap) : AbstractComponent() {
    private val ticksPerInch = 1.0
    private val leftMotor = hMap.get(DcMotorEx::class.java, "left_lift")
    private var leftOffset: Int = 0
    private val rightMotor = hMap.get(DcMotorEx::class.java, "right_lift")
    private var rightOffset: Int = 0
    var positionControlEnabled = false

    companion object {
        var initialized = false
            private set
        val coeffs = PIDCoefficients(0.004, 0.0, 0.0004)
        var fallPower = 0.01
        var slowFallPower = 0.01
        private var leftPosition = 0
        private var rightPosition = 0

        val lowJunction = 300
        val mediumJunction = 700
        val highJunction = 1165
    }
    val positionController: PIDFController = PIDFController(coeffs)

    init {
        leftMotor.direction = DcMotorSimple.Direction.REVERSE
        rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        leftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        positionController.tolerance = 200.0
    }
    private val emergencySensor = hMap.get(RevTouchSensor::class.java, "emergency_toucher")

    override fun update() {
        if (positionControlEnabled) {
            val position = getPosition()
            val output = positionController.update(position)
            setPower(output)
            CommandScheduler.telemetry.addLine("updating motors")
            CommandScheduler.telemetry.addData("lift output", output)
        }
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

    fun getVelocity() = 0.5 * (leftMotor.velocity + rightMotor.velocity)

    fun init(): Command = FunctionalCommand(
        init = {
            positionControlEnabled = false
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

    fun goToPosition(position: Int, maxVel: Double = 500.0, accel: Double = 300.0): Command {
        if (position.toDouble() epsilonEquals getPosition()) return Command.emptyCommand()
        return if (position >= getPosition()) {
            val motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(getPosition(), getVelocity(), accel),
                MotionState(position.toDouble(), 0.0, accel),
                maxVel,
                accel + 1.0
            )
            TimeCommand { t, _ ->
                val target = motionProfile[t].x
                CommandScheduler.telemetry.addData("target", target)
                    .addData("current Position", getPosition())
                    .addData("duration", motionProfile.duration())
                positionController.targetPosition = target
                (t > motionProfile.duration() && positionController.isAtSetPoint())
            }.onInit {
                leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                positionControlEnabled = true
            }.onEnd {
                if (emergencySensor.isPressed) {
                    positionControlEnabled = false
                    setPower(0.0)
                }
            }.setInterruptable(true).requires(this)
        } else FunctionalCommand(
            init = {
                positionControlEnabled = false
                leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                setPower(fallPower)
            }, execute = {
                if (abs(position - getPosition()) <= 400 || getVelocity() > maxVel) setPower(slowFallPower)
            }, end = {
                leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                if (position > 10) {
                    positionController.targetPosition = position.toDouble()
                    positionControlEnabled = true
                }
            }, isFinished = {
                abs(position - getPosition()) <= 50
            }, isInterruptable = true, requirements = setOf(this)
        )
    }

    fun goToHeight(height: Double): Command = goToPosition((height * ticksPerInch).roundToInt())
}
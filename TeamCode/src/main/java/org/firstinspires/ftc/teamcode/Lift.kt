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
import kotlin.math.*

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
//        val coeffs = PIDCoefficients(0.006, 0.0, 0.0004)
        val coeffs = PIDCoefficients(0.004)
//        var kVup = 0.0007
//        var kVdown = 0.0007
//        var kAup = 0.0
//        var kAdown = 0.0
        var fallAccel = -4000.0
        var slowTime = 0.4
        var fallPower = 0.001

        @JvmField var lowJunction = 450.0
        @JvmField var mediumJunction = 700.0
        @JvmField var highJunction = 1200.0
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

    var currentVelocity: Double? = null
    override fun update() {
        if (positionControlEnabled) {
            val position = getPosition()
            val velocity = currentVelocity
            val output = when {
                velocity == null -> 0.21
                velocity >= 0.0 -> velocity * 0.0006 + 0.21
                velocity < 0.0 -> fallPower
                else -> 0.21
            } + positionController.update(position)
            setPower(max(output, 0.0))
//            telem.addData("lift target", positionController.targetPosition)
//                .addData("lift actual", position)
//                .addData("lift targetVel", currentVelocity ?: 0.0)
//                .addData("lift actualVel", getVelocity())
//                .addData("lift output", output)
        }
    }

    fun setPower(power: Double) {
        leftMotor.power = power
        rightMotor.power = power
    }

    fun resetEncoders() {
        leftOffset = -leftMotor.currentPosition
        rightOffset = -rightMotor.currentPosition
    }

    fun getPosition() =
        0.5 * (leftMotor.currentPosition + leftOffset + rightMotor.currentPosition + rightOffset)

    fun getVelocity() = 0.5 * (leftMotor.velocity + rightMotor.velocity)

    fun init(): Command = FunctionalCommand(
        init = {
            positionControlEnabled = false
            leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            setPower(fallPower)
        },
        execute = {
            telem.addLine("resetting lift!!")
            telem.addData("emergency sensor pressed", emergencySensor.isPressed)
            telem.addData("motor power", leftMotor.power)
        },
        isFinished = { emergencySensor.isPressed },
        end = {
            setPower(-0.1)
            initialized = true
        },
        isInterruptable = false,
//        requirements = setOf(this)
    ) then WaitCommand(0.2).onEnd {
        setPower(0.0)
        resetEncoders()
    }

    fun goToPosition(position: Double, maxVel: Double = 1200.0, accel: Double = 1000.0): Command =
        Command.select(this) {
            val actual = getPosition()
            if (position epsilonEquals actual) return@select Command.emptyCommand()
            val currentVel = currentVelocity ?: getVelocity()

            if (position > actual) {
                val motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                    MotionState(getPosition(), currentVel, accel),
                    MotionState(position, 0.0, accel),
                    maxVel,
                    accel + 1.0
                )
                TimeCommand { t, _ ->
                    val targetState = motionProfile[t]
                    telem.addData("profile duration", motionProfile.duration())
                    positionController.targetPosition = targetState.x
                    currentVelocity = targetState.v
                    (t > motionProfile.duration())
                }.onInit {
                    leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                    rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                    positionControlEnabled = true
                }.onEnd {
                    currentVelocity = null
                    if (emergencySensor.isPressed) {
                        positionControlEnabled = false
                        setPower(0.0)
                    }
                }.setInterruptable(true).requires(this)
            } else {
                val root = sqrt(currentVel.pow(2) + 2 * fallAccel * (position - actual))
                val fallTime = listOf(
                    (-currentVel + root) / fallAccel,
                    (-currentVel - root) / fallAccel
                ).filter { it >= 0 }.maxOrNull()
                if (fallTime == null || fallTime < slowTime) {
                    val motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                        MotionState(getPosition(), currentVel, accel),
                        MotionState(position, 0.0, accel),
                        200.0,
                        accel + 1.0
                    )
                    TimeCommand { t, _ ->
                        val targetState = motionProfile[t]
                        telem.addData("profile duration", motionProfile.duration())
                        telem.addData("fallTime", fallTime)
                        positionController.targetPosition = targetState.x
                        currentVelocity = targetState.v
                        (t > motionProfile.duration())
                    }.onInit {
                        leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                        rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                        positionControlEnabled = true
                    }.onEnd {
                        currentVelocity = null
                        if (emergencySensor.isPressed) {
                            positionControlEnabled = false
                            setPower(0.0)
                        }
                    }.setInterruptable(true).requires(this)
                } else TimeCommand { t, _ ->
                    telem.addData("fall time", fallTime)
                    setPower(if (t < fallTime - slowTime) 0.0 else 0.1)
                    (t > fallTime || emergencySensor.isPressed)
                }.onInit {
                    leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                    rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                    positionControlEnabled = false
                }.onEnd {
                    currentVelocity = null
                    if (emergencySensor.isPressed) {
                        positionControlEnabled = false
                        setPower(0.0)
                    } else {
                        positionControlEnabled = true
                        positionController.targetPosition = position
                    }
                }.setInterruptable(true).requires(this)
            }
        }

    fun goToHeight(height: Double): Command = goToPosition(height * ticksPerInch)
}
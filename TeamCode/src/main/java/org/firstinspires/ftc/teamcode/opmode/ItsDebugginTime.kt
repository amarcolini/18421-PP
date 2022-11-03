package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.CommandScheduler
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.hardware.CRServo
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.Servo
import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Bot

@TeleOp(name = "Debug", group = "test")
@JoosConfig
class ItsDebugginTime : LinearOpMode() {
    private lateinit var robot: Bot

    companion object {
        var leftLiftPower = 0.0
        var rightLiftPower = 0.0
        var bothLiftPower = 0.0
            set(value) {
                leftLiftPower = value
                rightLiftPower = -value
                field = value
            }
        var leftArmPosition = 0.0
        var rightArmPosition = 0.0
        var armPosition = 0.0
            set(value) {
                leftArmPosition = value
                rightArmPosition = 1 - value
                field = value
            }

        var intakeServoPower = 0.0
    }

    override fun runOpMode() {
        val intake = CRServo(hardwareMap, "intake")
        val coneToucher = hardwareMap.get(RevTouchSensor::class.java, "cone_toucher")
        val emergencyToucher = hardwareMap.get(RevTouchSensor::class.java, "emergency_toucher")
        val rightLift = Motor(hardwareMap, "right_lift", Motor.Kind.GOBILDA_30)
        val leftLift = Motor(hardwareMap, "left_lift", Motor.Kind.GOBILDA_30)
        val rightArm = Servo(hardwareMap, "right_arm")
        val leftArm = Servo(hardwareMap, "left_arm")

        waitForStart()

        while (opModeIsActive() && !isStopRequested && !gamepad1.a) {
            intake.power = intakeServoPower
            rightLift.power = rightLiftPower
            leftLift.power = leftLiftPower
            leftArm.position = leftArmPosition
            rightArm.position = rightArmPosition
            CommandScheduler.telemetry
                .addData("cone???", coneToucher.isPressed)
                .addData("emergency???", emergencyToucher.isPressed)
            CommandScheduler.telemetry.update()
        }

        intake.power = 0.0
        rightLift.power = 0.0
        leftLift.power = 0.0
    }
}
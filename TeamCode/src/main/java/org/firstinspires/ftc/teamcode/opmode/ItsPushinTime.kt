package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.CommandScheduler
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.MotorGroup
import com.amarcolini.joos.hardware.drive.MecanumDrive
import com.amarcolini.joos.trajectory.constraints.MecanumConstraints
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "PushTest", group = "test")
class ItsPushinTime : LinearOpMode() {
    override fun runOpMode() {
        val drive = MecanumDrive(
            MotorGroup(hardwareMap, Motor.Kind.GOBILDA_312,
                "front_left" to false,
                "back_left" to false,
                "back_right" to false,
                "front_right" to false
            ), Imu(hardwareMap, "imu"),
            MecanumConstraints(
                trackWidth = 12.0,
                lateralMultiplier = 1.0,
                maxVel = 60.0,
                maxAccel = 30.0,
                maxAngVel = 180.deg,
                maxAngAccel = 180.deg
            )
        )
        drive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)

        waitForStart()

        var forwardTicks = 0.0
        var lateralTicks = 0.0
        while (opModeIsActive() && !isStopRequested) {
            drive.update()
            if (CommandScheduler.gamepad!!.p1.a.justActivated) {
                val ticks = drive.motors.motors.map(Motor::currentPosition)
                forwardTicks = ticks.average()
                lateralTicks = 0.25 * (-ticks[0] + ticks[1] - ticks[2] + ticks[3])
            }
            CommandScheduler.telemetry
                .addData("forwardTicks", forwardTicks)
                .addData("lateralTicks", lateralTicks)
            CommandScheduler.telemetry.update()
        }
    }
}
package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.MotorGroup
import com.amarcolini.joos.hardware.drive.MecanumDrive
import com.amarcolini.joos.trajectory.constraints.MecanumConstraints
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "DriveTest", group = "test")
class ItsDrivinTime : CommandOpMode() {
    override fun preInit() {
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
        register(drive)

        map({ gamepad.p1.leftStickChanged || gamepad.p1.rightStickChanged }, {
            val leftStick = gamepad.p1.getLeftStick()
            drive.setWeightedDrivePower(
                Pose2d(
                    -leftStick.y,
                    -leftStick.x,
                    -gamepad.p1.getRightStick().x
                )
            )
        })
    }
}
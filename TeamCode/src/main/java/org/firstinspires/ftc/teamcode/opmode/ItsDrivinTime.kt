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
import org.firstinspires.ftc.teamcode.TestDrive

@TeleOp(name = "DriveTest", group = "test")
class ItsDrivinTime : CommandOpMode() {
    override fun preInit() {
        val drive = TestDrive(hardwareMap)
        register(drive)

        schedule({
            val leftStick = gamepad.p1.getLeftStick()
            drive.setWeightedDrivePower(
                Pose2d(
                    -leftStick.y,
                    -leftStick.x,
                    -gamepad.p1.getRightStick().x
                )
            )
        }, true)
    }
}
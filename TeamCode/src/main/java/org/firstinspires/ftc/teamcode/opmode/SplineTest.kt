package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.TestDrive

@Autonomous
class SplineTest : CommandOpMode() {
    private lateinit var command: Command

    override fun preInit() {
        val drive = TestDrive(hardwareMap)
        register(drive)

        val pose = Pose2d(24.0, 24.0, 0.deg)
        command = drive.followTrajectory(
            drive.builder(Pose2d())
                .splineTo(pose.vec(), pose.heading)
                .build()
        ) then drive.followTrajectory(
            drive.builder(pose, true)
                .splineTo(Vector2d(0.0, 0.0), 180.deg)
                .build()
        )
    }

    override fun preStart() {
        schedule(command)
    }
}
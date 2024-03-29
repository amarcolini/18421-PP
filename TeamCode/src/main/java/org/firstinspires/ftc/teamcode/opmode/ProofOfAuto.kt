package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.*
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.AprilTagPipeline
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.Lift
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation


@Autonomous
@Disabled
class ProofOfAuto : CommandOpMode() {
    private lateinit var robot: Bot

    override fun preInit() {
        robot = registerRobot(Bot())
        robot.drive.initializeIMU()
        robot.drive.poseEstimate = Pose2d()

        schedule(
            SequentialCommand(
                true,
            robot.arm.down() and robot.intake::close
            )
        )
        schedule(true) {
            if (requiring(robot.lift) == null) telem.addLine("ready!")
        }
    }

    override fun preStart() {
        cancelAll()

        robot.lift.positionControlEnabled = true

        schedule(
            SequentialCommand(
                true,
                robot.arm.rest() and robot.drive.followTrajectory(
                    robot.drive.builder()
                        .turn((-30).deg)
                        .build()
                ),
                Command.select { robot.lift.goToPosition(Lift.highJunction) },
                robot.arm.goToAngle(250.deg) and WaitCommand(2.0),
                InstantCommand(robot.intake::open),
                robot.arm.rest() and Command.select { robot.lift.goToPosition(0.0) }
            )
        )
    }
}
package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.*
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.util.deg
import com.amarcolini.joos.util.epsilonEquals
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.AprilTagPipeline
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.Lift
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

@Disabled
@Autonomous(name = "Score ON THE RIGHT")
class ScoreRight : CommandOpMode() {
    private lateinit var robot: Bot

    //    private lateinit var camera: OpenCvWebcam
    private lateinit var pipeline: AprilTagPipeline

    override fun preInit() {
        robot = registerRobot(Bot())
        robot.drive.initializeIMU()
        robot.drive.poseEstimate = Pose2d()

//        camera = OpenCvCameraFactory.getInstance().createWebcam(
//            hardwareMap.get(
//                WebcamName::class.java, "webcam"
//            )
//        )
        pipeline = AprilTagPipeline()
//        camera.setPipeline(pipeline)
//        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
//            override fun onOpened() {
//                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT)
//                dashboard?.startCameraStream(camera, 60.0)
//            }
//            override fun onError(errorCode: Int) {}
//        })

        schedule(
            SequentialCommand(
                true,
                robot.arm.down() and robot.intake::close,
                WaitCommand(0.2),
                robot.arm.rest()
            )
        )
        schedule({
            if (requiring(robot.lift) == null) telem.addLine("ready!")
        }, true)
    }

    override fun preStart() {
        cancelAll()
        robot.drive.poseEstimate = Pose2d(-63.0, -36.0, 0.deg)
//        val detections = pipeline.latestDetections
//        camera.closeCameraDevice()

//        val strafeDistance = when (detections.firstOrNull()?.id) {
//            0 -> 24.0
//            2 -> -24.0
//            else -> 0.0
//        }
        val strafeDistance = listOf(-24.0, 0.0, 24.0).random()

        robot.lift.positionControlEnabled = false

        schedule(
            SequentialCommand(
                true,
                robot.arm.rest() and robot.drive.followTrajectory {
                    forward(48.0)
                    build()
                } and robot.lift.goToPosition(1165),
                robot.drive.followFromLast {
                    turn((-50).deg)
                    forward(2.0)
                    build()
                }, robot.arm.goToAngle(250.deg),
                WaitCommand(0.3),
                InstantCommand(robot.intake::open),
                WaitCommand(0.5) then robot.arm.rest(),
                (Command.select { robot.lift.goToPosition(0) }) and
                        (WaitCommand(1.0) then robot.drive.followFromLast {
                            turn((50).deg)
                            lineToLinearHeading(Pose2d(
                                Vector2d(-12.0, -36.0 + strafeDistance), 0.deg))
                            build()
                        }),
                Command.of {
                    Bot.poseStorage = robot.drive.poseEstimate
                    requestOpModeStop()
                }
//                Command.select { robot.lift.goToPosition(Lift.highJunction) },
//                robot.arm.goToAngle(250.deg) and WaitCommand(2.0),
//                robot.intake.eject(),
//                robot.arm.rest() and Command.select { robot.lift.goToPosition(0) }
            )
        )
    }
}
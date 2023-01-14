package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.command.*
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.AprilTagPipeline
import org.firstinspires.ftc.teamcode.Bot
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera2

@Autonomous(name = "1+5 High (Right)")
class TheAutoRight : TheAuto() {
    override var transform: Pose2d.() -> Pose2d = { Pose2d(x, -y, -heading) }
}

@Config
@Autonomous(name = "1+5 High (Left)")
open class TheAuto : CommandOpMode() {
    protected open var transform: Pose2d.() -> Pose2d = { this }
    private lateinit var robot: Bot

    private lateinit var camera: OpenCvCamera
    private lateinit var pipeline: AprilTagPipeline

    companion object {
        @JvmField var startPose = Pose2d(-63.0, -36.0, 0.deg)
        @JvmField var initialScorePose = Pose2d(-12.0, -39.0, 40.deg)
        @JvmField var stackPose = Pose2d(-12.0, -63.0, (90).deg)
        @JvmField var scorePose = Pose2d(-9.0, 33.0, 40.deg)
    }

    override fun preInit() {
        robot = registerRobot(Bot())
        robot.drive.initializeIMU()
        robot.drive.poseEstimate = Pose2d()

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId)
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED)

        pipeline = AprilTagPipeline()
        camera.setPipeline(pipeline)
        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(800, 600, OpenCvCameraRotation.SIDEWAYS_LEFT)
                dashboard?.startCameraStream(camera, 60.0)
            }
            override fun onError(errorCode: Int) {}
        })

        schedule(robot.arm.rest() and robot.intake::close)
        schedule({
            if (requiring(robot.lift) == null) telem.addLine("ready!")
        }, true)
    }

    override fun preStart() {
        cancelAll()
        robot.drive.poseEstimate = startPose.transform()
        val detections = pipeline.latestDetections
        camera.pauseViewport()
        camera.closeCameraDeviceAsync {}

        val initialScorePose = initialScorePose.transform()
        val stackPose = stackPose.transform()
        val scorePose = scorePose.transform()
        val parkPose = Pose2d(-12.0, when (detections.firstOrNull()?.id) {
            0 -> 24.0
            2 -> 0.0
            else -> -24.0
        }, 0.deg)

        robot.lift.positionControlEnabled = false

        schedule(
            SequentialCommand(
                true,
                ParallelCommand(true, robot.arm.rest(), robot.drive.followTrajectory {
                    lineToSplineHeading(initialScorePose)
                    build()
                }, robot.lift.goToPosition(1165)),
                robot.arm.out(),
                InstantCommand(robot.intake::open),
                robot.arm.rest() race WaitCommand(0.3),
                robot.drive.followFromLast {
                    lineToSplineHeading(stackPose)
                    build()
                } and robot.lift.goToPosition(600),
                //TODO: pick up stack cones and score
                WaitCommand(1.0), robot.drive.followFromLast {
                    lineToLinearHeading(parkPose)
                    build()
                },
                Command.of {
                    Bot.poseStorage = robot.drive.poseEstimate
                    requestOpModeStop()
                }
            )
        )
    }
}
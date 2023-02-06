package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.*
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.amarcolini.joos.util.epsilonEquals
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.AprilTagPipeline
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.Lift
import org.openftc.easyopencv.*

@Autonomous(name = "Just Park")
class JustPark : CommandOpMode() {
    private lateinit var robot: Bot

    private lateinit var camera: OpenCvCamera
    private lateinit var pipeline: AprilTagPipeline
    private var tagId = 2

    override fun preInit() {
        robot = registerRobot(Bot())
        robot.drive.initializeIMU()
        robot.drive.poseEstimate = Pose2d()

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "webcam"))
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED)

        pipeline = AprilTagPipeline()
        camera.setPipeline(pipeline)
        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(640, 480)
                dashboard?.startCameraStream(camera, 60.0)
            }
            override fun onError(errorCode: Int) {}
        })

        schedule(robot.arm.rest() and robot.intake::close)
        schedule(true) {
            if (requiring(robot.lift) == null) telemetry.addLine("ready!")
            val id =  pipeline.latestDetections.getOrNull(0)?.id
            telemetry.addData("tag id", id)
            if (id != null) tagId = id
            telemetry.update()
        }
        initLoop = true
    }

    override fun preStart() {
        cancelAll()
        robot.drive.poseEstimate = Pose2d()
        camera.pauseViewport()
        camera.closeCameraDeviceAsync {}

        val strafeDistance = when (tagId) {
            0 -> 24.0
            42 -> -24.0
            else -> 0.0
        }

        robot.lift.positionControlEnabled = false

        schedule(
            robot.drive.followTrajectory {
                forward(24.0)
                if (strafeDistance != 0.0) strafeLeft(strafeDistance)
                build()
            } then ::requestOpModeStop
        )
    }
}
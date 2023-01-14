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

    override fun preInit() {
        robot = registerRobot(Bot())
        robot.drive.initializeIMU()
        robot.drive.poseEstimate = Pose2d()

        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK)
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
        schedule({
            if (requiring(robot.lift) == null) telem.addLine("ready!")
        }, true)
    }

    override fun preStart() {
        cancelAll()
        robot.drive.poseEstimate = Pose2d()
        val detections = pipeline.latestDetections
        camera.closeCameraDeviceAsync {}

        val strafeDistance = when (detections.firstOrNull()?.id) {
            0 -> 24.0
            2 -> 0.0
            else -> -24.0
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
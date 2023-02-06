package org.firstinspires.ftc.teamcode.opmode

import android.content.Context
import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.command.*
import com.amarcolini.joos.dashboard.ConfigHandler
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.*
import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMetaAndClass
import org.firstinspires.ftc.teamcode.AprilTagPipeline
import org.firstinspires.ftc.teamcode.Bot
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera2

@JoosConfig
@Autonomous(name = "1+5 High (Right)")
open class TheAuto(private val transform: Pose2d.() -> Pose2d = { this }) : CommandOpMode() {
    private lateinit var robot: Bot

    private lateinit var camera: OpenCvCamera
    private lateinit var pipeline: AprilTagPipeline
    private var tagId: Int = 2

    companion object {
        var startPose = Pose2d(-63.0, -36.0, 0.deg)
        var initialScorePose = Pose2d(-14.0, -32.0, 40.deg)
        var stackPose = Pose2d(-12.0, -63.0, (90).deg)
        var scorePose = Pose2d(-9.0, 33.0, 40.deg)

        @JvmStatic
        @OpModeRegistrar
        fun get(opModeManager: OpModeManager) {
            opModeManager.register(OpModeMeta.Builder().run {
                name = "1+5 High (Left)"
                flavor = OpModeMeta.Flavor.AUTONOMOUS
                source = OpModeMeta.Source.ANDROID_STUDIO
                build()
            }, TheAuto { Pose2d(x - 4.0, -y, -heading) })
        }
    }

    final override fun preInit() {
        robot = registerRobot(Bot())
        robot.drive.initializeIMU()
        robot.drive.poseEstimate = Pose2d()

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "webcam"), cameraMonitorViewId)
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

        schedule(Command.of {
            robot.arm.servo.position = 0.0
            robot.intake.open()
        } wait 1.0 then robot.intake::close)
        schedule(true) {
            if (requiring(robot.lift) == null) telem.addData("Ready!", "")
            val id =  pipeline.latestDetections.getOrNull(0)?.id
            telem.addData("tag id", id)
            if (id != null) tagId = id
        }
        initLoop = true
    }

    final override fun preStart() {
        cancelAll()
        robot.drive.poseEstimate = startPose.transform()

        val initialScorePose = initialScorePose.transform()
        val stackPose = stackPose.transform()
        val scorePose = scorePose.transform()
        val parkPose = Pose2d(0.0, when (tagId) {
            0 -> 24.0
            42 -> -24.0
            else -> 0.0
        }, 0.deg) + Pose2d(-12.0, -36.0, 0.deg).transform()

        robot.lift.positionControlEnabled = false

        schedule(
            SequentialCommand(
                true,
                ParallelCommand(true, robot.arm.rest(), robot.drive.followTrajectory {
                    forward(24.0)
                    splineTo(initialScorePose.vec(), initialScorePose.heading)
                    build()
                }, robot.lift.goToPosition(1165.0)),
                robot.arm.outDown() and WaitCommand(1.0),
                InstantCommand(robot.intake::open) and WaitCommand(0.3),
                robot.arm.rest() wait 0.5,
                robot.drive.followTrajectory(initialScorePose) {
                    lineToSplineHeading(parkPose)
                    build()
                } and robot.lift.goToPosition(0.0),
                //TODO: pick up stack cones and score
                Command.of {
                    Bot.poseStorage = robot.drive.poseEstimate
                    requestOpModeStop()
                }
            )
        )
    }
}
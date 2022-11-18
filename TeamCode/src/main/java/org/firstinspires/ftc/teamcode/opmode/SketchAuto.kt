package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.*
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.AprilTagPipeline
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.Lift
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

@Autonomous(name = "Our only auto")
class SketchAuto : CommandOpMode() {
    private lateinit var robot: Bot
    private lateinit var camera: OpenCvWebcam
    private lateinit var pipeline: AprilTagPipeline

    override fun preInit() {
        robot = registerRobot(Bot())
        robot.drive.initializeIMU()
        robot.drive.poseEstimate = Pose2d()

        camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(
                WebcamName::class.java, "webcam"
            )
        )
        pipeline = AprilTagPipeline()
        camera.setPipeline(pipeline)
        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT)
                dashboard?.startCameraStream(camera, 60.0)
            }
            override fun onError(errorCode: Int) {}
        })

        schedule(
            SequentialCommand(
                true,
                robot.arm.down() and robot.intake::slurp,
                WaitCommand(0.2),
                InstantCommand(robot.intake::stop),
                robot.arm.rest()
            )
        )
        schedule({
            if (requiring(robot.lift) == null) telem.addLine("ready!")
        }, true)
    }

    override fun preStart() {
        cancelAll()
        val detections = pipeline.latestDetections
        camera.closeCameraDevice()

        val parkPose = when (detections[0].id) {
            0 -> Pose2d()
            2 -> Pose2d()
            else -> Pose2d()
        }

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
                robot.intake.eject(),
                robot.arm.rest() and Command.select { robot.lift.goToPosition(0) }
            )
        )
    }
}
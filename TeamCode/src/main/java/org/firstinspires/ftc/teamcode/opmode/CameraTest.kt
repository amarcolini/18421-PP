package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.command.CommandScheduler
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.AprilTagPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous(group = "test")
class CameraTest : LinearOpMode() {
    override fun runOpMode() {
        val camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(
                WebcamName::class.java, "webcam"
            )
        )
        val pipeline = AprilTagPipeline()
        camera.setPipeline(pipeline)
        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT)
                FtcDashboard.getInstance()?.startCameraStream(camera, 60.0)
            }
            override fun onError(errorCode: Int) {}
        })

        waitForStart()

        while(opModeIsActive()) {
            val detections = pipeline.latestDetections
            CommandScheduler.telemetry.addData("id", detections[0].id)
            CommandScheduler.telemetry.update()
        }
    }
}
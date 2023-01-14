package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.AprilTagPipeline
import org.openftc.easyopencv.*
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener

@TeleOp
class BackCameraPls : LinearOpMode() {
    override fun runOpMode() {
        val camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK)
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED)

        val pipeline = AprilTagPipeline()
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(960, 720, OpenCvCameraRotation.SIDEWAYS_LEFT)
                camera.setPipeline(pipeline)
                FtcDashboard.getInstance()?.startCameraStream(camera, 0.0)
            }

            override fun onError(errorCode: Int) {
            }
        })

        waitForStart()

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance()?.telemetry)
        while(opModeIsActive()) {
            Thread.sleep(200)
            telemetry.addData("detections", pipeline.latestDetections)
            telemetry.addData("id", pipeline.latestDetections.getOrNull(0)?.id)
            telemetry.update()
        }
    }
}

@TeleOp
class FrontCameraPls : LinearOpMode() {
    override fun runOpMode() {
        val camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.FRONT)
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED)

        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(960, 720)
                FtcDashboard.getInstance()?.startCameraStream(camera, 0.0)
            }

            override fun onError(errorCode: Int) {
            }
        })

        waitForStart()

        while(opModeIsActive()) {
            Thread.sleep(200)
        }
    }
}
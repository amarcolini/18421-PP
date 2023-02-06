package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.command.CommandScheduler
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.control.PIDFController
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.util.NanoClock
import com.amarcolini.joos.util.deg
import com.amarcolini.joos.util.rad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.*
import java.lang.Double.min
import kotlin.math.PI
import kotlin.math.pow

@JoosConfig
class PoleAlign(hMap: HardwareMap, private val drive: TestDrive, id: String = "webcam") {
    private val camera: OpenCvCamera = OpenCvCameraFactory.getInstance()
        .createWebcam(hMap.get(WebcamName::class.java, "webcam"))
    private val pipeline = PoleAlignPipeline()
    private val headingController = PIDFController(headingCoeffs)
    private val forwardController = PIDFController(forwardCoeffs)

    companion object {
        var headingCoeffs = PIDCoefficients(2.0, 0.0)
        var targetPosition = 467.0
        var forwardCoeffs = PIDCoefficients(0.0, 0.0)
        var targetWidth = 98.0
    }

    private fun widthToDist(width: Double) = 1147.31 * 0.666 / width
    private fun posToHeading(pos: Double) = ((1 - (pos) / 640.0) * -33.0 + 7.0).deg

    private val targetHeading = posToHeading(targetPosition)
    private val targetDist = widthToDist(targetWidth)

    init {
        headingController.setInputBounds(-PI, PI)
        headingController.setOutputBounds(-0.3, 0.3)

        forwardController.setOutputBounds(-0.3, 0.3)

        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED)
        camera.setPipeline(pipeline)
        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
                FtcDashboard.getInstance()?.startCameraStream(camera, 0.0)
            }

            override fun onError(errorCode: Int) {
            }
        })
    }

    fun finish() {
        camera.closeCameraDeviceAsync {}
    }

    private val clock: NanoClock = NanoClock.system()
    private var lastUpdate: Double? = null
    private var timeout: Double? = null
    private var targetPos: Vector2d = Vector2d()
    fun update() {
        val dt = lastUpdate?.let { clock.seconds() - it } ?: Double.POSITIVE_INFINITY
        val width = pipeline.poleWidth
        val pos = pipeline.polePos
        if (width == null || pos == null) {
            val time = timeout
            if (time != null && clock.seconds() - time > 0.5) {
                forwardController.reset()
                headingController.reset()
                lastUpdate = null
            }
            return
        }
        timeout = clock.seconds()
        if (dt <= 0.5) return
        val currentHeading = posToHeading(pos)
        val currentDist = widthToDist(width)
        targetPos = Vector2d.polar(currentDist, 0.rad)
        drive.poseEstimate = Pose2d(0.0, 0.0, currentHeading)
        forwardController.targetPosition = targetDist
        headingController.targetPosition = (currentHeading - targetHeading).radians
        lastUpdate = clock.seconds()
    }

    fun getPower(): Pose2d {
        update()
        if (lastUpdate == null) return Pose2d()
        val delta = (targetPos - drive.poseEstimate.vec())
        CommandScheduler.telemetry.addData("targetDist", targetDist)
            .addData("targetHeading", targetHeading)
            .addData("computed distance", delta.norm())
            .addData("computed angle", delta.angle())
            .addData("computed heading delta", delta.angle() - drive.poseEstimate.heading)
        headingController.targetPosition = delta.angle().radians + targetHeading.radians
        val power = Pose2d(
            -forwardController.update(delta.norm()),
            0.0,
            headingController.update(drive.poseEstimate.heading.radians)
        )
        CommandScheduler.telemetry.addData("alignPower", power)
        return power
    }
}

//@JoosConfig
class PoleAlignPipeline : OpenCvPipeline() {
    companion object {
        var process = false
    }

    private val telem = CommandScheduler.telemetry
    var poleWidth: Double? = null
    var polePos: Double? = null
    var lowerBound = Scalar(31.2, 66.6, 126.1)
    var upperBound = Scalar(100.6, 235.2, 255.0)
    private val temp = Mat()
    private val poleFilter = Scalar(31.2, 66.6, 85.0) to Scalar(112.0, 235.2, 255.0)
    private val poles = Mat()
    private val redConeFilter = Scalar(114.8, 102.0, 0.0) to Scalar(137.4, 255.0, 255.0)
    private val redCones = Mat()
    private val blueConeFilter = Scalar(0.0, 102.0, 0.0) to Scalar(22.7, 255.0, 255.0)
    private val blueCones = Mat()

    private fun overlayColor(input: Mat, dst: Mat, mask: Mat, color: Scalar) {
        Mat(input.size(), input.type(), color).copyTo(dst, mask)
    }

    override fun processFrame(input: Mat): Mat {
        Imgproc.cvtColor(input, temp, Imgproc.COLOR_BGR2HSV)

        Core.inRange(temp, poleFilter.first, poleFilter.second, poles)
//        overlayColor(input, input, poles, new Scalar(256, 238, 0));

        Core.inRange(temp, redConeFilter.first, redConeFilter.second, redCones)
//        overlayColor(input, input, redCones, new Scalar(255, 0, 0));

        Core.inRange(temp, blueConeFilter.first, blueConeFilter.second, blueCones)
//        overlayColor(input, input, blueCones, new Scalar(0, 0, 255));

        if (process) return poles
        Core.add(redCones, blueCones, temp)
        Imgproc.erode(temp, temp, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, Size(5.0, 5.0)))
        Imgproc.dilate(temp, temp, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, Size(5.0, 5.0)))

        Imgproc.erode(poles, poles, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, Size(5.0, 5.0)))
        Imgproc.dilate(poles, poles, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, Size(5.0, 5.0)))
//        Imgproc.Canny(temp, temp, thresh1, thresh2);
        val coneContours = ArrayList<MatOfPoint>()
        Imgproc.findContours(temp, coneContours, Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.drawContours(input, coneContours, -1, Scalar(0.0, 255.0, 0.0), 1)
        coneContours.forEach { contour ->
            val rect = Imgproc.boundingRect(contour)
            val moments = Imgproc.moments(contour)
            Imgproc.circle(
                input,
                Point(moments.m10 / moments.m00, moments.m01 / moments.m00),
                3,
                Scalar(255.0, 0.0, 0.0),
                -1
            )
            Imgproc.rectangle(input, rect, Scalar(255.0, 0.0, 0.0), 3)
        }
        val poleContours = ArrayList<MatOfPoint>()
        Imgproc.findContours(poles, poleContours, Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.drawContours(input, poleContours, -1, Scalar(0.0, 255.0, 0.0), 1)
        poleWidth = null
        polePos = null
        poleContours.forEach { contour ->
            val rect = Imgproc.minAreaRect(MatOfPoint2f(*contour.toArray()))
            val minSide = min(rect.size.width, rect.size.height)
            if (minSide < 40) return@forEach
            val polePos = rect.center.x - rect.size.height * 0.5 * rect.angle.deg.sin() * if(rect.size.height > rect.size.width) 1.0 else -1.0
            poleWidth = minSide
            this.polePos = polePos
            Imgproc.polylines(input, arrayOfNulls<Point>(4).run {
                rect.points(this)
                listOf(MatOfPoint(*this))
            }, true, Scalar(0.0, 0.0, 255.0), 3)
            Imgproc.line(input, Point(polePos, 0.0), Point(polePos, input.size().height), Scalar(255.0, 0.0, 0.0), 3)
        }
        return input
    }
}
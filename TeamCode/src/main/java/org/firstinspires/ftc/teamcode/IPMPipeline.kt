package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.dashboard.JoosConfig
import org.opencv.core.*
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

@JoosConfig
class IPMPipeline : OpenCvPipeline() {

    enum class Mode{
            FRAME,
            WARPED
    }

    companion object {
        var mode = Mode.FRAME
        val rectCoords = RectCoords(
            90 to 150,
            212 to 155,
            58 to 240,
            256 to 240
        )
        val objectSize = Size(8.5, 11.0)
        var pixelScale = 5.0
    }
    
    data class RectCoords(
        var tl: Point,
        var tr: Point,
        var bl: Point,
        var br: Point
    ) {
        constructor(
            tl: Pair<Number, Number>,
            tr: Pair<Number, Number>,
            bl: Pair<Number, Number>,
            br: Pair<Number, Number>
        ) : this(
            Point(tl.first.toDouble(), tl.second.toDouble()),
            Point(tr.first.toDouble(), tr.second.toDouble()),
            Point(bl.first.toDouble(), bl.second.toDouble()),
            Point(br.first.toDouble(), br.second.toDouble()),
        )
    }
    
    fun getTransformationMatrix(rectCoords: RectCoords, objectSize: Size, imageSize: Size, pixelScale: Double): Mat {
        val (tl, tr, bl, br) = rectCoords
        val src = MatOfPoint2f(tl, tr, bl, br)
        val dst = MatOfPoint2f(
            Point((imageSize.width/2)-(objectSize.width*pixelScale/2),imageSize.height -(objectSize.height*pixelScale)),
         Point((imageSize.width/2)+(objectSize.width*pixelScale/2),imageSize.height -(objectSize.height*pixelScale)),
         Point((imageSize.width/2)-(objectSize.width*pixelScale/2), imageSize.height),
         Point((imageSize.width/2)+(objectSize.width*pixelScale/2), imageSize.height)
        )
        return Imgproc.getPerspectiveTransform(src, dst)
    }

//    public Vector2d getPosition(Point src, Size size){
//        point = new Mat()
//        point.put(0,0,src.x,src.y)
//        point = transformationMatrix.inv().mul(point)
//        return new Vector2d(point.get(0,0)[0], point.get(0,0)[1])
//    }

    override fun processFrame(input: Mat): Mat {
        when (mode) {
            Mode.FRAME -> {
                val points = listOf(MatOfPoint(
                    rectCoords.tl,
                    rectCoords.tr,
                    rectCoords.bl,
                    rectCoords.br,
                ))
                Imgproc.polylines(input, points, true, Scalar(0.0, 255.0, 0.0))
                return input
            }
            Mode.WARPED -> {
                val transformationMatrix = getTransformationMatrix(rectCoords, objectSize, input.size(), pixelScale)
                Imgproc.warpPerspective(input, input, transformationMatrix, input.size(), Imgproc.INTER_LINEAR)
                return input
            }
            else -> return input
        }
    }
}
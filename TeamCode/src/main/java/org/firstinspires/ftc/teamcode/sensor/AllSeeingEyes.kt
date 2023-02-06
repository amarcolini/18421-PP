package org.firstinspires.ftc.teamcode.sensor

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.localization.Localizer
import com.amarcolini.joos.util.rad
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.PI
import kotlin.math.roundToInt

@JoosConfig
class AllSeeingEyes(hMap: HardwareMap, val localOdometry: Localizer): AbstractComponent(), Localizer {
    private val backEye = hMap.get(AnalogInput::class.java, "back_eye")
    private val frontEye = hMap.get(AnalogInput::class.java, "front_eye")
    private val leftEye = hMap.get(AnalogInput::class.java, "left_eye")
    private val rightEye = hMap.get(AnalogInput::class.java, "right_eye")

    private val sensorMap = listOf(
        frontEye to Pose2d(7.5, -4.25, 0.rad),
        backEye to Pose2d(-7.25, 4.25, PI.rad),
        leftEye to Pose2d(-.5, 6.0, (PI * 0.5).rad),
        rightEye to Pose2d(-.5, -6.0, (-PI * 0.5).rad)
    )
    
    private val fieldBoundaries = listOf(
        Line(-halfField, -halfField, -halfField, halfField),
        Line(-halfField, halfField, halfField, halfField),
        Line(halfField, halfField, halfField, -halfField),
        Line(halfField, -halfField, -halfField, -halfField)
    )
    
    private fun getVectorTransform(initial: List<Vector2d>): (Angle) -> List<Vector2d> = { heading -> initial.map {
        it.rotated(heading)
    } }

    private val algorithm: SensorAlgorithm = Informed()

    interface SensorAlgorithm {
        /**
         * @return the new pose (only if it is confident enough)
         */
        fun update(vectorTransform: (Angle) -> List<Vector2d>, headingEstimate: Angle): Pose2d?
    }

    private fun getDistance(sensor: AnalogInput) = sensor.voltage * 204.7244 / sensor.maxVoltage

    override val poseVelocity: Pose2d? = null
    override var poseEstimate = Pose2d()
    override fun update() {
        localOdometry.update()

        telem.fieldOverlay().setStroke("blue")
        val vectors = sensorMap.map { (sensor, pose) ->
            val distance = getDistance(sensor)
            telem.addData("d(${pose.heading.degrees.roundToInt()}°)", distance)
            val start = pose.vec()
            val end = Vector2d.polar(distance, pose.heading) + start
            telem.fieldOverlay().strokeLine(start.x, start.y, end.x, end.y)
            end
        }
        val transform = getVectorTransform(vectors)
        val result = algorithm.update(transform, localOdometry.poseEstimate.heading)
        if (result != null) {
            poseEstimate = result
        }

        val headingVec = poseEstimate.headingVec() * 10.0
        telem.fieldOverlay().setStroke("red")
            .strokeLine(0.0, 0.0, headingVec.x, headingVec.y)
    }
}

package org.firstinspires.ftc.teamcode.sensor

import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.serialization.format
import com.amarcolini.joos.util.DoubleProgression
import com.amarcolini.joos.util.abs
import com.amarcolini.joos.util.deg
import kotlin.math.max
import kotlin.math.min

class Informed : AllSeeingEyes.SensorAlgorithm {
    data class ProcessResult(
        val line: AxisLine,
        val count: Int,
        val range: Double
    )

    private fun computeFitness(heading: Angle, vectors: List<Vector2d>): Pair<Pose2d, Double> {
        val horizontals = ArrayList<AxisLine>()
        val verticals = ArrayList<AxisLine>()

        vectors.forEach { vector ->
            val top = halfField - vector.x
            val bottom = -halfField - vector.x
            val left = -halfField - vector.y
            val right = halfField - vector.y

            val trimmedHorizontalRange = max(-halfField, left)..min(halfField, right)
            if (top in -halfField..halfField) horizontals += AxisLine(
                top,
                trimmedHorizontalRange
            )
            if (bottom in -halfField..halfField) horizontals += AxisLine(
                bottom,
                trimmedHorizontalRange
            )

            val trimmedVerticalRange = max(-halfField, bottom)..min(halfField, top)
            if (left in -halfField..halfField) verticals += AxisLine(
                left,
                trimmedVerticalRange
            )
            if (right in -halfField..halfField) verticals += AxisLine(
                right,
                trimmedVerticalRange
            )
        }

//        val processAxisLines = { lines: ArrayList<AxisLine> ->
//            val newLines = ArrayList<ProcessResult>()
//            while (!lines.isEmpty()) {
//                val currentLine = lines[0]
//                var p = currentLine.pos
//                var r = currentLine.range
//                var count = 1.0
//                var posRange = p..p
//                lines.drop(1).forEach {
//                    if (abs(it.pos - p) <= 8.0 && r intersects it.range) {
//                        p = (it.pos + p) * 0.5
////                        r = min(r.start, it.range.start)..max(r.endInclusive, it.range.endInclusive)
//                        val intersect = r.intersect(it.range)
//                        r = (intersect.start - 4.0)..(intersect.endInclusive + 4.0)
//                        lines.remove(it)
//                        count++
//                        posRange = min(posRange.start, it.pos)..max(posRange.endInclusive, it.pos)
//                    }
//                }
//                newLines += ProcessResult(AxisLine(p, r), count, (posRange.endInclusive - posRange.start))
//                lines.remove(currentLine)
//            }
//            newLines
//        }

        val processAxisLines = { lines: ArrayList<AxisLine> ->
            val newLines = ArrayList<ProcessResult>()
            val currentLines = ArrayList<AxisLine>()
            var lastPos: Double? = null
            lines.sortBy { it.pos }
            println(lines.map { it.pos })
            for (line in lines) {
                if (lastPos?.let { line.pos - it <= 12.0 } == false) {
                    println(currentLines)
                    val newLine = if (currentLines.size % 2 != 0)
                        currentLines[(currentLines.size - 1) / 2]
                    else {
                        val line1 = currentLines[(currentLines.size - 1) / 2]
                        val line2 = currentLines[(currentLines.size - 1) / 2 + 1]
                        AxisLine(
                            (line1.pos + line2.pos) * 0.5,
                            min(line1.range.start, line2.range.start)
                                    ..max(line1.range.endInclusive, line2.range.endInclusive)
                        )
                    }
                    val maxRange = currentLines.last().pos - currentLines.first().pos
                    newLines += ProcessResult(newLine, currentLines.size, maxRange)
                    currentLines.clear()
                }
                currentLines += line
                lastPos = line.pos
            }
            println(currentLines)
            val newLine = if (currentLines.size % 2 != 0)
                currentLines[(currentLines.size - 1) / 2]
            else {
                val line1 = currentLines[(currentLines.size - 1) / 2]
                val line2 = currentLines[(currentLines.size - 1) / 2 + 1]
                AxisLine(
                    (line1.pos + line2.pos) * 0.5,
                    min(line1.range.start, line2.range.start)
                            ..max(line1.range.endInclusive, line2.range.endInclusive)
                )
            }
            val maxRange = currentLines.last().pos - currentLines.first().pos
            newLines += ProcessResult(newLine, currentLines.size, maxRange)
            currentLines.clear()
            newLines
        }

        val newHorizontals = processAxisLines(horizontals.toMutableList() as ArrayList)
        val newVerticals = processAxisLines(verticals.toMutableList() as ArrayList)

        val intersections = newVerticals.flatMap { (yLine, yCount, yRange) ->
            newHorizontals.mapNotNull { (xLine, xCount, xRange) ->
                if (xLine.pos in yLine.range && yLine.pos in xLine.range) {
                    val xFitness = xCount - 0.1 * xRange
                    val yFitness = yCount - 0.1 * yRange
                    Vector2d(xLine.pos, yLine.pos) to (xFitness + yFitness + if (xCount + yCount < 3) -1.0 else 0.0)
                } else null
            }
        }
        val (pos, fitness) = intersections.maxByOrNull { it.second }!!
        return Pose2d(pos, heading) to fitness
    }

    private var currentRange = 0.0..180.0
    private var lastHeading: Angle? = null
    override fun update(vectorTransform: (Angle) -> List<Vector2d>, headingEstimate: Angle): Pose2d? {
        if (lastHeading?.let { abs(it - headingEstimate) <= 2.deg } != true) {
            val deg = headingEstimate.degrees
            currentRange = (deg - 5.0)..(deg + 5.0)
        }
        val step = (currentRange.endInclusive - currentRange.start) / 9
        val prog = DoubleProgression(currentRange.start, step, 10)
        val result = prog.map {
            computeFitness(it.deg, vectorTransform(it.deg))
        }
        val (estimate, fitness) = result.maxByOrNull { it.second }!!
        val newHeading = estimate.heading.degrees
        currentRange = (newHeading - step)..(newHeading + step)
        lastHeading = headingEstimate
        return if (fitness > 3.5) estimate else null
    }
}
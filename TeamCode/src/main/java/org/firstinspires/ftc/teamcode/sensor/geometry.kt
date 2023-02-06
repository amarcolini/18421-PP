package org.firstinspires.ftc.teamcode.sensor

import com.amarcolini.joos.geometry.Vector2d

data class Line(val start: Vector2d, val end: Vector2d) {
    constructor(startX: Double, startY: Double, endX: Double, endY: Double) : this(
        Vector2d(startX, startY), Vector2d(endX, endY)
    )

    fun length() = (end - start).norm()

    operator fun plus(vector: Vector2d) = Line(start + vector, end + vector)

    /**
     * Returns the point of intersection between two lines, or null if they do not intersect.
     */
    infix fun intersect(other: Line): Vector2d? {
        val p = start
        val r = (end - start)
        val q = other.start
        val s = (other.end - other.start)
        if (r cross s == 0.0) return null

        val a = (q - p)
        val b = 1 / (r cross s)

        val t = (a cross s) * b
        val u = (a cross r) * b

        return if (t in 0.0..1.0 && u in 0.0..1.0) p + r * t
        else null
    }
}

data class AxisLine(val pos: Double, val range: ClosedRange<Double>)

const val halfField = 72.0
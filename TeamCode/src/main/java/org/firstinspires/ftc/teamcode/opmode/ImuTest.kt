package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.util.rad
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import kotlin.math.abs
import kotlin.math.sign

@Autonomous(group = "test")
@Disabled
class ImuTest : CommandOpMode() {
    override fun preInit() {
        val imu = Imu(hardwareMap, "imu")
        imu.autoDetectUpAxis()

        schedule({
            val heading = imu.imu.angularOrientation.toAngleUnit(AngleUnit.RADIANS).toAxesOrder(AxesOrder.XYZ)
            val gravity = imu.imu.gravity
            val result = listOf(gravity.xAccel, gravity.yAccel, gravity.zAccel).zip(Imu.Axis.values())
            val max = result.maxByOrNull { abs(it.first) }
            telem.addData("heading", imu.heading)
                .addData("max", max ?: "null")
                .addData("headingVelocity", imu.headingVelocity)
                .addData("correctedZ", Math.toDegrees(heading.thirdAngle.toDouble()))
                .addData("axis", imu.axis)
        }, true)
    }
}
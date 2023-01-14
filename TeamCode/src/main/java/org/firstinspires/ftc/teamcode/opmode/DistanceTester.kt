package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.Component
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.localization.Localizer
import com.amarcolini.joos.util.NanoClock
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.sensor.AllSeeingEyes

@Autonomous(group = "test")
class DistanceTester : CommandOpMode() {
    private lateinit var eyes: AllSeeingEyes

    override fun preInit() {
        eyes = AllSeeingEyes(hardwareMap, object: Localizer {
            override var poseEstimate: Pose2d = Pose2d()
            override val poseVelocity: Pose2d? = null
            override fun update() {}
        })
        register(eyes)

        val hubs = hardwareMap.getAll(LynxModule::class.java)
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        val clock = NanoClock.system()
        var lastTimeStamp = clock.seconds()

        register(Component.of {
            hubs.forEach { it.clearBulkCache() }
            telem.addData("loop time (hZ)", 1.0 / (clock.seconds() - lastTimeStamp))
            FtcDashboard.getInstance()?.sendTelemetryPacket(TelemetryPacket().apply { addLine("working") })
            lastTimeStamp = clock.seconds()
        })
    }
}
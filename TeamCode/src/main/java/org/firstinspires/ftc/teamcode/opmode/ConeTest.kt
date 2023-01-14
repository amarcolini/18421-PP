package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl

@TeleOp(group = "test")
@JoosConfig
@Disabled
class ConeTest : OpMode() {
    private lateinit var conePeeper: RevColorSensorV3

    companion object {
        var gain: Double = 30.0
        var lightOn: Boolean = true
    }

    override fun init() {
        conePeeper = hardwareMap.get(RevColorSensorV3::class.java, "cone_peeper")
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
    }

    override fun loop() {
        val distance = conePeeper.getDistance(DistanceUnit.CM)
        telemetry.addData("peep distance", distance)
        telemetry.addData("light detected", conePeeper.lightDetected)
        telemetry.addData("led enabled", conePeeper.isLightOn)
        telemetry.addData("has cone???", distance < 2.5)
        telemetry.addData("gain", conePeeper.gain)
        val color = conePeeper.normalizedColors
        val formatted = "rgba(${color.red}, ${color.green}, ${color.blue}, ${color.alpha})"
        telemetry.addData("cone color???",
            if (color.red > 0.6) "red"
            else if (color.blue > 0.3) "blue"
            else "unknown"
        )
        telemetry.addData("colors", "<font color='${color.toColor()}'>█</font> $formatted")
        conePeeper.gain = gain.toFloat()
        conePeeper.enableLed(lightOn)
    }
}
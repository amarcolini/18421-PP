package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.AnalogSensor

@Autonomous(group = "test")
class DistanceTester : OpMode() {
    private lateinit var sensor: AnalogInput

    override fun init() {
        sensor = hardwareMap.get(AnalogInput::class.java, "back_eyes")
    }

    override fun loop() {
        val voltage = sensor.voltage
        telemetry.addData("raw voltage", voltage)
            .addData("computed distance", voltage * 520 / sensor.maxVoltage)
    }
}
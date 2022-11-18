package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap

class AllSeeingEyes(hMap: HardwareMap): AbstractComponent() {
    private val backEye = hMap.get(AnalogInput::class.java, "back_eye")
    private val frontEye = hMap.get(AnalogInput::class.java, "front_eye")
    private val leftEye = hMap.get(AnalogInput::class.java, "left_eye")
    private val rightEye = hMap.get(AnalogInput::class.java, "right_eye")

    private fun getDistance(sensor: AnalogInput) = sensor.voltage * 204.7244 / sensor.maxVoltage

    override fun update() {
    }
}
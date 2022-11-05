package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.InstantCommand
import com.amarcolini.joos.hardware.CRServo
import com.amarcolini.joos.hardware.Servo
import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.HardwareMap

class Intake(hMap: HardwareMap) : AbstractComponent() {
    private val servo = CRServo(hMap, "intake")
    private val coneToucher = hMap.get(RevTouchSensor::class.java, "cone_toucher")

    val hasCone get() = coneToucher.isPressed

    fun stop(): Command = InstantCommand { servo.power = 0.0 }

    fun slurp(): Command = InstantCommand { servo.power = -1.0 }

    fun eject(): Command = InstantCommand { servo.power = 1.0 } wait 1.0 then stop()
}
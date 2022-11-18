package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.InstantCommand
import com.amarcolini.joos.hardware.CRServo
import com.amarcolini.joos.hardware.Servo
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class Intake(hMap: HardwareMap) : AbstractComponent() {
    private val servo = CRServo(hMap, "intake")
    private val conePeeper = hMap.get(RevColorSensorV3::class.java, "cone_peeper")

    val hasCone get() = conePeeper.getDistance(DistanceUnit.CM) < 2.5

    fun stop() { servo.power = 0.0 }

    fun slurp() { servo.power = -1.0 }

    fun eject(): Command = InstantCommand { servo.power = 1.0 } wait 1.0 then(::stop)
}
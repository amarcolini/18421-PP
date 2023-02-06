package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.InstantCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.hardware.CRServo
import com.amarcolini.joos.hardware.Servo
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@JoosConfig
class Intake(hMap: HardwareMap) : AbstractComponent() {
    private val servo = Servo(hMap, "intake")
    private val conePeeper = hMap.get(RevColorSensorV3::class.java, "cone_peeper")

    val hasCone get() = conePeeper.getDistance(DistanceUnit.CM) < 10.5
    val peepDist get() = conePeeper.getDistance(DistanceUnit.CM)

    companion object {
        var closedPosition = 0.2
        var openPosition = 0.35
    }
    var isOpen = false
        private set

    fun close() {
        servo.position = closedPosition
        isOpen = false
    }

    fun open() {
        servo.position = openPosition
        isOpen = true
    }
}
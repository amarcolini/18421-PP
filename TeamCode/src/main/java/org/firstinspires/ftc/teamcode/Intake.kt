package org.firstinspires.ftc.teamcode

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

//@Config
class Intake(hMap: HardwareMap) : AbstractComponent() {
    private val servo = Servo(hMap, "intake")
    private val conePeeper = hMap.get(RevColorSensorV3::class.java, "cone_peeper")

    val hasCone get() = conePeeper.getDistance(DistanceUnit.CM) < 2.5

    companion object {
        @JvmField var closedPosition = 0.0
        @JvmField var openPosition = 0.5
    }

    fun close() { servo.position = closedPosition }

    fun open() { servo.position = openPosition }
}
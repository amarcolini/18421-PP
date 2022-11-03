package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.InstantCommand
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.hardware.Servo
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.hardware.HardwareMap

class Arm(hMap: HardwareMap) : AbstractComponent() {
    private val leftServo = Servo(hMap, "left_arm", 180.deg)
    private val rightServo = Servo(hMap, "right_arm", 180.deg)

    init {
        subcomponents += listOf(leftServo, rightServo)
    }

    fun setAngle(angle: Angle): Command = InstantCommand {
        leftServo.angle = angle
        rightServo.angle = angle
    }
}
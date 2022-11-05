package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.InstantCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.hardware.Servo
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.hardware.HardwareMap

@JoosConfig
class Arm(hMap: HardwareMap) : AbstractComponent() {
    private val leftServo = Servo(hMap, "left_arm", 300.deg)
    private val rightServo = Servo(hMap, "right_arm", 300.deg)

    companion object {
        //0 is down, 1 is up, 0.23 is rest down (69 degrees)
        var rest = 69.deg
        var down = 0.deg
        var out = 250.deg
        var outDown = 270.deg
    }

    init {
        rightServo.reversed = true
        subcomponents += listOf(leftServo, rightServo)
    }

    fun goToAngle(angle: Angle): Command = (leftServo.goToAngle(angle, 70.0) and rightServo.goToAngle(angle,70.0))
        .setInterruptable(false)
        .requires(this)

    fun rest() = goToAngle(rest)

    fun down() = goToAngle(down)

    fun out() = goToAngle(out)

    fun outDown() = goToAngle(outDown)
}
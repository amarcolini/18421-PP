package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.InstantCommand
import com.amarcolini.joos.command.WaitCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.hardware.Servo
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.hardware.HardwareMap

@JoosConfig
class Arm(hMap: HardwareMap) : AbstractComponent() {
    val servo = Servo(hMap, "right_arm", 270.deg)

    companion object {
        //0 is down, 1 is up, 0.23 is rest down (69 degrees)
        var rest = 35.deg
        var down = 0.deg
        var out = 195.deg
        var outDown = 265.deg
        var outScore = 230.deg
    }

    init {
        servo.reversed = true
        subcomponents += servo
    }

    fun goToAngle(angle: Angle): Command = Command.select(this) {
        WaitCommand((servo.angle - angle).abs().normDelta() / 60.deg * 0.2).onInit {
            servo.angle = angle
        }
    }

    fun rest() = goToAngle(rest)

    fun down() = goToAngle(down)

    fun out() = goToAngle(out)

    fun outDown() = goToAngle(outDown)

    fun outScore() = goToAngle(outScore)
}
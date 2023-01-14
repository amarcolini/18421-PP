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

@Config
class Arm(hMap: HardwareMap) : AbstractComponent() {
    private val servo = Servo(hMap, "right_arm", 270.deg)

    companion object {
        //0 is down, 1 is up, 0.23 is rest down (69 degrees)
        @JvmField var rest = 30.deg
        @JvmField var down = 0.deg
        @JvmField var out = 210.deg
        @JvmField var outDown = 260.deg
    }

    init {
        servo.reversed = true
        subcomponents += servo
    }

    fun goToAngle(angle: Angle): Command = WaitCommand(1.0)
        .onInit {
            servo.angle = angle
        }
        .requires(this)

    fun rest() = goToAngle(rest)

    fun down() = goToAngle(down)

    fun out() = goToAngle(out)

    fun outDown() = goToAngle(outDown)
}
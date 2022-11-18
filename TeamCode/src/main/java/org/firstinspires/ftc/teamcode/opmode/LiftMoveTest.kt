package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Lift

@Autonomous(name = "LiftMoveTest", group = "test")
@JoosConfig
class LiftMoveTest : CommandOpMode() {
    private lateinit var lift: Lift

    companion object {
        var position = 800
        var maxVel = 600.0
        var accel = 300.0
        var decel = 100.0
    }

    override fun preInit() {
        lift = Lift(hardwareMap)
        register(lift)

        schedule(lift.init())
    }

    override fun preStart() {
        schedule(lift.goToPosition(position, maxVel, accel))
    }
}
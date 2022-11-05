package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.InstantCommand
import com.amarcolini.joos.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.Lift
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "DriveControl", group = "competition")
class DriveControl : CommandOpMode() {
    private lateinit var robot: Bot

    override fun preInit() {
        robot = registerRobot(Bot())

        if (!Lift.initialized) schedule(robot.lift.init())
    }

    override fun preStart() {
        var armExtended = false
        val liftPositions = listOf(1000, 2000, 3000)
        var index = 2
        var liftUp = false

        map({ gamepad.p1.x.justActivated }, Command.select {
            if (armExtended) robot.arm.rest().onEnd { armExtended = false }
            else robot.arm.out().onEnd { armExtended = true }
        }.requires(robot.arm))
        map({ gamepad.p1.a.justActivated && robot.lift.controller.isAtSetPoint() && !liftUp }, Command.select {
            val command = if (robot.intake.hasCone) {
                robot.intake.eject() wait 1.0 then robot.intake.stop()
            } else {
                val armDownCommand = if (armExtended) robot.arm.outDown() else robot.arm.down()
                val armUpCommand = if (armExtended) robot.arm.out() else robot.arm.rest()
                (armDownCommand and robot.intake.slurp()).runUntil(robot.intake::hasCone) race WaitCommand(
                    3.0
                ) then (robot.intake.stop() and armUpCommand)
            }
            command
                .setInterruptable(false)
                .requires(robot.arm, robot.intake, robot.lift)
        })

        map(gamepad.p1.y::justActivated, InstantCommand {
            if (liftUp) robot.lift.controller.targetPosition = 0.0
            else robot.lift.controller.targetPosition = liftPositions[index].toDouble()
            liftUp = !liftUp
        }.requires(robot.lift))
        map(gamepad.p1.dpad_up::justActivated, InstantCommand {
            index++
            index = min(index, liftPositions.size - 1)
            if (liftUp) robot.lift.controller.targetPosition = liftPositions[index].toDouble()
        })
        map(gamepad.p1.dpad_down::justActivated, InstantCommand {
            index--
            index = max(index, 0)
            if (liftUp) robot.lift.controller.targetPosition = liftPositions[index].toDouble()
        })

        //TODO: Drive code here
    }
}
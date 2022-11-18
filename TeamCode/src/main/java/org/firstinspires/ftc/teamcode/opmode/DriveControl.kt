package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.*
import com.amarcolini.joos.gamepad.MultipleGamepad
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.Lift
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "DriveControl", group = "competition")
class DriveControl : CommandOpMode() {
    private lateinit var robot: Bot
    private lateinit var pad: MultipleGamepad

    override fun preInit() {
        pad = MultipleGamepad(gamepad1, gamepad2)
        robot = registerRobot(Bot())
        register(Component.of {
            pad.update()
        })

        if (!Lift.initialized) schedule(robot.lift.init())
        schedule(robot.arm.rest())
    }

    override fun preStart() {
        var armExtended = false
        val liftPositions = listOf(300, 700, 1165)
        var index = 2
        var liftUp = false
        var hasWorkedAtAll = false

        robot.lift.positionControlEnabled = true

        map({ pad.p1.x.justActivated }, Command.select {
            hasWorkedAtAll = true
            val angle = if (!armExtended)
                if (!liftUp) 300.deg else 250.deg
            else 69.deg
            armExtended = !armExtended
            robot.arm.goToAngle(angle).setInterruptable(false)
        }.requires(robot.arm))
        map({ pad.p1.a.justActivated && robot.lift.positionController.isAtSetPoint() && !liftUp && !robot.intake.hasCone }, Command.select {
            hasWorkedAtAll = true
            if (armExtended) return@select Command.emptyCommand()
            val armDownCommand = robot.arm.down()
            val armUpCommand = robot.arm.rest()
            (armDownCommand and robot.intake::slurp) wait(0.2) then
                    (armUpCommand and robot.intake::stop)
                .setInterruptable(false)
                .requires(robot.arm, robot.intake, robot.lift)
        })
        map(pad.p1.b::justActivated, robot.intake.eject())

        map(pad.p1.y::justActivated, Command.select {
            hasWorkedAtAll = true
            val position = if (liftUp) 0
            else liftPositions[index]
            var command = robot.lift.goToPosition(position)
            if (armExtended) {
                command = command and if (liftUp) robot.arm.goToAngle(300.deg) else robot.arm.goToAngle(250.deg)
            }
            liftUp = !liftUp
            command
        }.requires(robot.lift).setInterruptable(true))
        map(pad.p1.dpad_up::justActivated, Command.select {
            hasWorkedAtAll = true
            index++
            index = min(index, liftPositions.size - 1)
            if (liftUp) robot.lift.goToPosition(liftPositions[index])
            else Command.emptyCommand()
        }.setInterruptable(true))
        map(pad.p1.dpad_down::justActivated, Command.select {
            hasWorkedAtAll = true
            index--
            index = max(index, 0)
            if (liftUp) robot.lift.goToPosition(liftPositions[index])
            else Command.emptyCommand()
        }.setInterruptable(true))

        schedule({
            val leftStick = pad.p1.getLeftStick()
            robot.drive.setWeightedDrivePower(
                Pose2d(
                    -leftStick.y,
                    -leftStick.x,
                    -pad.p1.getRightStick().x
                )
            )
            CommandScheduler.telemetry.addData("lift at rest", robot.lift.positionController.isAtSetPoint())
                .addData("worked", hasWorkedAtAll)
                .addData("index", index)
                .addData("liftUp", liftUp)
                .addData("armExtended", armExtended)
        }, true)
    }
}
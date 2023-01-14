package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.*
import com.amarcolini.joos.gamepad.MultipleGamepad
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Arm
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
        robot.drive.initializeIMU()
        register(Component.of {
            pad.update()
        })

//        if (!Lift.initialized) schedule(robot.lift.init())
        schedule(robot.arm.rest())
    }

    override fun preStart() {
        var armExtended = false
        val liftPositions = listOf(Lift.lowJunction, Lift.mediumJunction, Lift.highJunction)
        var index = 2
        var liftUp = false
        cancelAll()

        robot.lift.positionControlEnabled = true

        map(pad.p1.x::justActivated, Command.select(robot.arm) {
            armExtended = !armExtended
            if (armExtended)
                if (!liftUp) robot.arm.outDown() else robot.arm.out()
            else robot.arm.rest()
        }.requires(robot.arm))
        map({ pad.p1.a.justActivated && robot.lift.positionController.isAtSetPoint() && !(armExtended || liftUp || robot.intake.hasCone) }, Command.select(robot.arm) {
            val armDownCommand = robot.arm.down()
            val armUpCommand = robot.arm.rest()
            (armDownCommand and robot.intake::close) wait(0.1) then armUpCommand
                .setInterruptable(false)
                .requires(robot.arm, robot.intake, robot.lift)
        })
        map(pad.p1.b::justActivated, robot.intake::open)

        map(pad.p1.y::justActivated, Command.select(robot.lift) {
            val position = if (liftUp) {
                index = 2
                0
            }
            else liftPositions[index]
            var command: Command = robot.lift.goToPosition(position)
            if (armExtended) {
                command = command and robot.arm.goToAngle(if (!liftUp) 270.deg else 230.deg)
            }
            liftUp = !liftUp
            command
        })
        map({pad.p1.right_bumper.justActivated && robot.lift.currentVelocity == null && liftUp}, Command.select(robot.lift) {
            robot.lift.goToPosition(max(0, (robot.lift.getPosition() + 200).toInt()))
        })
        map({pad.p1.left_bumper.justActivated && robot.lift.currentVelocity == null && liftUp}, Command.select(robot.lift) {
            robot.lift.goToPosition(min(liftPositions.last(), (robot.lift.getPosition() - 20).toInt()))
        })
        var hasWorked = false
        map({pad.p1.right_trigger.justActivated && robot.lift.currentVelocity == null && liftUp}, Command.select(robot.lift) {
            hasWorked = true
            robot.lift.goToPosition(liftPositions[index])
        })
        map(pad.p1.dpad_left::justActivated, Command.select(robot.lift) {
            index = 0
            if (liftUp) robot.lift.goToPosition(liftPositions[index])
            else Command.emptyCommand()
        }.setInterruptable(true))
        map(pad.p1.dpad_up::justActivated, Command.select(robot.lift) {
            index = 1
            if (liftUp) robot.lift.goToPosition(liftPositions[index])
            else Command.emptyCommand()
        }.setInterruptable(true))
        map(pad.p1.dpad_right::justActivated, Command.select(robot.lift) {
            index = 2
            if (liftUp) robot.lift.goToPosition(liftPositions[index])
            else Command.emptyCommand()
        }.setInterruptable(true))

        schedule({
            val leftStick = pad.p1.getLeftStick()
//            robot.drive.setWeightedDrivePower(
//                Pose2d(
//                    Vector2d(-leftStick.y, -leftStick.x).rotated(-robot.drive.poseEstimate.heading),
//                    -pad.p1.getRightStick().x
//                )
//            )
            robot.drive.setWeightedDrivePower(
                Pose2d(
                    -leftStick.y,
                    -leftStick.x,
                    -pad.p1.getRightStick().x
                )
            )
            CommandScheduler.telemetry.addData("lift at rest", robot.lift.positionController.isAtSetPoint())
                .addData("index", index)
                .addData("currentVelocity", robot.lift.currentVelocity)
                .addData("liftUp", liftUp)
                .addData("hasWorked", hasWorked)
                .addData("armExtended", armExtended)
        }, true)
    }
}
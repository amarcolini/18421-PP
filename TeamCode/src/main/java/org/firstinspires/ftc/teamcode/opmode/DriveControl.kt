package org.firstinspires.ftc.teamcode.opmode

import org.firstinspires.ftc.teamcode.PoleAlign
import com.amarcolini.joos.command.*
import com.amarcolini.joos.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.Lift
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "DriveControl", group = "competition")
class DriveControl : CommandOpMode() {
    private lateinit var robot: Bot
    private lateinit var poleAlign: PoleAlign

    override fun preInit() {
        robot = registerRobot(Bot())
        robot.drive.initializeIMU()
        poleAlign = PoleAlign(hardwareMap, robot.drive)

//        if (!Lift.initialized) schedule(robot.lift.init())
        schedule(robot.arm.rest() and robot.intake::open)
        initLoop = true
    }

    override fun preStart() {
        var armExtended = false
        val liftPositions = listOf(Lift.lowJunction, Lift.mediumJunction, Lift.highJunction)
        var index = 2
        var liftUp = false
        cancelAll()
        robot.drive.poseEstimate = Pose2d()

        robot.lift.positionControlEnabled = true

        map({gamepad.p1.x.justActivated && !gamepad.p1.right_trigger.isActive }, Command.select(robot.arm) {
            armExtended = !armExtended
            if (armExtended)
                if (!liftUp) robot.arm.outDown() else robot.arm.out()
            else robot.arm.rest()
        }.requires(robot.arm))

        map({gamepad.p1.right_trigger.justActivated && armExtended && liftUp}, robot.arm.outScore())
        map({gamepad.p1.right_trigger.justDeactivated && armExtended && liftUp}, robot.arm.out())
        map({gamepad.p1.a.justActivated && robot.lift.positionController.isAtSetPoint() && !(armExtended || liftUp) }, Command.select(robot.arm) {
            val armDownCommand = robot.arm.down()
            val armUpCommand = robot.arm.rest()
            (armDownCommand and WaitCommand(0.2).then(robot.intake::close)) wait(0.2) then armUpCommand
                .setInterruptable(false)
                .requires(robot.arm, robot.intake, robot.lift)
        })
        map(gamepad.p1.b::justActivated, robot.intake::open)

        map(gamepad.p1.y::justActivated, Command.select(robot.lift) {
            val position = if (liftUp) {
                if (armExtended && robot.intake.isOpen) return@select Command.emptyCommand()
                index = 2
                0.0
            }
            else liftPositions[index]
            var command: Command = robot.lift.goToPosition(position)
            if (armExtended)
                command = command and if (!liftUp) robot.arm.out() else robot.arm.outDown()
            liftUp = !liftUp
            command
        })
        map({gamepad.p1.right_bumper.justActivated && robot.lift.currentVelocity == null && liftUp}, Command.select(robot.lift) {
            robot.lift.goToPosition(max(0.0, (robot.lift.getPosition() + 200.0)))
        })
        map({gamepad.p1.left_bumper.justActivated && robot.lift.currentVelocity == null && liftUp}, Command.select(robot.lift) {
            robot.lift.goToPosition(min(liftPositions.last(), (robot.lift.getPosition() - 20.0)))
        })
        map({gamepad.p1.right_trigger.justActivated && robot.lift.currentVelocity == null && liftUp}, Command.select(robot.lift) {
            robot.lift.goToPosition(liftPositions[index])
        })
        map(gamepad.p1.dpad_left::justActivated, Command.select(robot.lift) {
            index = 0
            if (liftUp) robot.lift.goToPosition(liftPositions[index])
            else Command.emptyCommand()
        }.setInterruptable(true))
        map(gamepad.p1.dpad_up::justActivated, Command.select(robot.lift) {
            index = 1
            if (liftUp) robot.lift.goToPosition(liftPositions[index])
            else Command.emptyCommand()
        }.setInterruptable(true))
        map(gamepad.p1.dpad_right::justActivated, Command.select(robot.lift) {
            index = 2
            if (liftUp) robot.lift.goToPosition(liftPositions[index])
            else Command.emptyCommand()
        }.setInterruptable(true))

        schedule(true) {
            val leftStick = gamepad.p1.getLeftStick()
//            robot.drive.setWeightedDrivePower(
//                Pose2d(
//                    Vector2d(-leftStick.y, -leftStick.x).rotated(-robot.drive.poseEstimate.heading),
//                    -pad.p1.getRightStick().x
//                )
//            )
            val coeff = if (armExtended && liftUp) {
                0.5
            } else 1.0
            robot.drive.setWeightedDrivePower(
                Pose2d(
                    -leftStick.y,
                    -leftStick.x,
                    -gamepad.p1.getRightStick().x
                ) * coeff
//                        + if (liftUp && armExtended) poleAlign.getPower() else Pose2d()
            )
            CommandScheduler.telemetry
                .addData("intake has cone", robot.intake.hasCone)
                .addData("peep distance", robot.intake.peepDist)
                .addData("robot heading", robot.drive.poseEstimate.heading)
        }
    }

    override fun postStop() {
        poleAlign.finish()
    }
}
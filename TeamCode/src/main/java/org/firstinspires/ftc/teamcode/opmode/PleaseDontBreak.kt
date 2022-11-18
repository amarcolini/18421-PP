package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.CommandScheduler
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Lift

@TeleOp(name = "LiftTest", group = "test")
@JoosConfig
class PleaseDontBreak : CommandOpMode() {
    private lateinit var lift: Lift

    companion object {
        val coefficients: PIDCoefficients = PIDCoefficients(0.0, 0.0, 0.0)
        var target: Int = 0
    }

    private var initialized = false
    override fun preInit() {
        lift = Lift(hardwareMap)
        lift.positionController.pid = coefficients
        register(lift)

        map(gamepad.p1.a::isActive, {
            lift.setPower(0.0)
            stopOpMode()
        })
        schedule(lift.init().onEnd { initialized = true })
    }

    override fun preStart() {
        lift.positionControlEnabled = true
        schedule({
            lift.positionController.targetPosition = target.toDouble()
            lift.positionController.pid = coefficients
            CommandScheduler.telemetry.addLine("working...")
            CommandScheduler.telemetry.addData("coefficients", lift.positionController.pid)
                .addData("target", lift.positionController.targetPosition)
                .addData("measured", lift.getPosition())
                .addData("initialized", initialized)
        }, true)
    }
}
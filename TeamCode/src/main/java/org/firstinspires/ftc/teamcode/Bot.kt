package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Component
import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.control.FeedforwardCoefficients
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.MotorGroup
import com.amarcolini.joos.hardware.drive.MecanumDrive
import com.amarcolini.joos.localization.MecanumLocalizer
import com.amarcolini.joos.trajectory.constraints.MecanumConstraints
import com.amarcolini.joos.util.deg
import com.qualcomm.hardware.lynx.LynxModule

class Bot : Robot() {
    val lift: Lift = Lift(hMap)
    val arm: Arm = Arm(hMap)
    val intake: Intake = Intake(hMap)
    val drive = TestDrive(hMap)

    companion object {
        var poseStorage: Pose2d = Pose2d()
    }

    init {
        val hubs = hMap.getAll(LynxModule::class.java)
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
        register(lift, arm, intake, drive, Component.of {
            hubs.forEach { it.clearBulkCache() }
        })
    }

    override fun init() {
        lift.resetEncoders()
        schedule(lift.init())
    }
}
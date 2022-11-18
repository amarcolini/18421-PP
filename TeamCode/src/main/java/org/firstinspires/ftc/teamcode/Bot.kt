package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.control.FeedforwardCoefficients
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.MotorGroup
import com.amarcolini.joos.hardware.drive.MecanumDrive
import com.amarcolini.joos.localization.MecanumLocalizer
import com.amarcolini.joos.trajectory.constraints.MecanumConstraints
import com.amarcolini.joos.util.deg

class Bot : Robot() {
    val lift: Lift = Lift(hMap)
    val arm: Arm = Arm(hMap)
    val intake: Intake = Intake(hMap)
    val drive = TestDrive(hMap)

    init {
        register(lift, arm, intake, drive)
    }

    override fun init() {
        schedule(lift.init())
    }
}
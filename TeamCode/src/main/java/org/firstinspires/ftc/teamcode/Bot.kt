package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Robot

class Bot : Robot() {
    val lift: Lift = Lift(hMap)
    val arm: Arm = Arm(hMap)
    val intake: Intake = Intake(hMap)

    override fun init() {
        schedule(lift.init())
    }
}
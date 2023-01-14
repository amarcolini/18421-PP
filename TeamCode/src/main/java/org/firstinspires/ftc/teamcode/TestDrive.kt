package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandScheduler
import com.amarcolini.joos.command.Component
import com.amarcolini.joos.command.FunctionalCommand
import com.amarcolini.joos.control.FeedforwardCoefficients
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.drive.AbstractMecanumDrive
import com.amarcolini.joos.drive.DriveSignal
import com.amarcolini.joos.followers.HolonomicPIDVAFollower
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.hardware.Imu
import com.amarcolini.joos.localization.MecanumLocalizer
import com.amarcolini.joos.trajectory.Trajectory
import com.amarcolini.joos.trajectory.TrajectoryBuilder
import com.amarcolini.joos.trajectory.constraints.*
import com.amarcolini.joos.util.deg
import com.amarcolini.joos.util.rad
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.PI
import kotlin.math.abs

@JoosConfig
class TestDrive(private val hMap: HardwareMap) : AbstractMecanumDrive(
    FeedforwardCoefficients(0.014, 0.001, 0.11453), 12.05, 12.05, 1.11
), Component {
    private var imu: Imu? = null
    private val frontLeft = hMap.get(DcMotorEx::class.java, "front_left")
    private val backLeft = hMap.get(DcMotorEx::class.java, "back_left")
    private val frontRight = hMap.get(DcMotorEx::class.java, "front_right")
    private val backRight = hMap.get(DcMotorEx::class.java, "back_right")

    companion object {
        val axialCoeffs = PIDCoefficients(4.0, 0.0, 0.005)
        val lateralCoeffs = PIDCoefficients(4.0, 0.0, 0.005)
        val headingCoeffs = PIDCoefficients(4.0, 0.0, 0.005)
    }

    init {
        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE

        listOf(frontLeft, backLeft, frontRight, backRight).forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        imu?.axis = Imu.Axis.Z
        imu?.reversed = false
    }
    
    private val inPerTick = (4 * PI * 0.94) / 537.7
    private val velConstraint = MinVelocityConstraint(
        MecanumVelocityConstraint(312.0 * 60 * 537.7 * inPerTick, 12.05, 12.05, 1.11),
        TranslationalVelocityConstraint(45.0),
        AngularVelocityConstraint(260.deg),
        AngularAccelVelocityConstraint(500.deg, 30.0)
    )
    private val accelConstraint = MinAccelerationConstraint(
        TranslationalAccelerationConstraint(30.0),
        AngularAccelerationConstraint(500.deg)
    )

    private val trajectoryFollower = HolonomicPIDVAFollower(
        axialCoeffs, lateralCoeffs, headingCoeffs,
        Pose2d(0.5, 0.5, 5.deg), 0.5
    )
    
    override val rawExternalHeading: Angle
        get() = imu?.heading ?: 0.rad

    fun initializeIMU() {
        imu = Imu(hMap, "imu")
    }

    override fun getExternalHeadingVelocity(): Angle? = imu?.headingVelocity

    override fun getWheelPositions(): List<Double> = listOf(
        frontLeft.currentPosition * inPerTick,
        backLeft.currentPosition * inPerTick,
        backRight.currentPosition * inPerTick,
        frontRight.currentPosition * inPerTick
    )

    override fun getWheelVelocities(): List<Double> = listOf(
        frontLeft.velocity * inPerTick,
        backLeft.velocity * inPerTick,
        backRight.velocity * inPerTick,
        frontRight.velocity * inPerTick
    )

    override fun setMotorPowers(
        frontLeft: Double,
        rearLeft: Double,
        rearRight: Double,
        frontRight: Double
    ) {
        this.frontLeft.power = frontLeft
        this.backLeft.power = rearLeft
        this.backRight.power = rearRight
        this.frontRight.power = frontRight
    }

    override fun update() {
        updatePoseEstimate()
        CommandScheduler.telemetry.addData("pose", poseEstimate)
            .addData("x", poseEstimate.x)
            .addData("y", poseEstimate.y)
            .addData("theta", poseEstimate.heading.radians)
            .addData("targetX", 0.0)
            .addData("targetY", 0.0)
            .addData("targetTheta", 0.0)
        if (trajectoryFollower.isFollowing()) {
            CommandScheduler.telemetry.drawSampledTrajectory(trajectoryFollower.trajectory)
            val target = trajectoryFollower.trajectory[trajectoryFollower.elapsedTime()]
            val targetVelocity = trajectoryFollower.trajectory.velocity(trajectoryFollower.elapsedTime())
            CommandScheduler.telemetry.drawRobot(target, "blue")
            CommandScheduler.telemetry.addData("targetPose", target)
                .addData("targetX", target.x)
                .addData("targetY", target.y)
                .addData("targetTheta", target.heading.radians)
        }
        CommandScheduler.telemetry.drawRobot(poseEstimate, "red")
    }

    fun followTrajectory(trajectory: Trajectory): Command = FunctionalCommand(
        init = { trajectoryFollower.followTrajectory(trajectory) },
        execute = { setDriveSignal(trajectoryFollower.update(poseEstimate, poseVelocity)) },
        end = {
            setDriveSignal(DriveSignal())
            lastTarget = trajectory.end()
              },
        isFinished = { !trajectoryFollower.isFollowing() },
        isInterruptable = true,
        requirements = setOf(this)
    )

    fun followTrajectory(startPose: Pose2d, startTangent: Angle, trajectory: TrajectoryBuilder.() -> Trajectory): Command = Command.select {
        followTrajectory(builder(startPose, startTangent).trajectory())
    }

    fun followTrajectory(startPose: Pose2d = poseEstimate, trajectory: TrajectoryBuilder.() -> Trajectory): Command = Command.select {
        followTrajectory(builder(startPose).trajectory())
    }

    var lastTarget: Pose2d? = null
    fun followFromLast(trajectory: TrajectoryBuilder.(Pose2d) -> Trajectory): Command = Command.select {
        val pose = lastTarget ?: poseEstimate
        followTrajectory(builder(pose).trajectory(pose))
    }

    fun followFromLast(startTangent: Angle, trajectory: TrajectoryBuilder.(Pose2d) -> Trajectory): Command = Command.select {
        val pose = lastTarget ?: poseEstimate
        followTrajectory(builder(pose, startTangent).trajectory(pose))
    }

    fun followFromLast(reversed: Boolean, trajectory: TrajectoryBuilder.(Pose2d) -> Trajectory): Command = Command.select {
        val pose = lastTarget ?: poseEstimate
        followTrajectory(builder(pose, reversed).trajectory(pose))
    }

    fun builder(startPose: Pose2d, startTangent: Angle = startPose.heading) = TrajectoryBuilder(
        startPose, startTangent, velConstraint, accelConstraint,
        260.deg, 180.deg, 180.deg
    )

    fun builder(startTangent: Angle = poseEstimate.heading) = TrajectoryBuilder(
        poseEstimate, startTangent, velConstraint, accelConstraint,
        260.deg, 180.deg, 180.deg
    )

    fun builder(startPose: Pose2d, reversed: Boolean) = TrajectoryBuilder(
        startPose, reversed, velConstraint, accelConstraint,
        260.deg, 180.deg, 180.deg
    )

    fun builder(reversed: Boolean) = TrajectoryBuilder(
        poseEstimate, reversed, velConstraint, accelConstraint,
        260.deg, 180.deg, 180.deg
    )

    fun setWeightedDrivePower(
        drivePower: Pose2d,
        xWeight: Double = 1.0,
        yWeight: Double = 1.0,
        headingWeight: Double = 1.0
    ) {
        var vel = drivePower

        if (abs(vel.x) + abs(vel.y) + abs(vel.heading.value) > 1) {
            val denom =
                xWeight * abs(vel.x) + yWeight * abs(vel.y) + headingWeight * abs(vel.heading.value)
            vel = Pose2d(vel.x * xWeight, vel.y * yWeight, vel.heading.value * headingWeight) / denom
        }

        setDrivePower(vel.copy(heading = vel.heading.value.rad))
    }
}
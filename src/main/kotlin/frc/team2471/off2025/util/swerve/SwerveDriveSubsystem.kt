package frc.team2471.off2025.util.swerve

import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveRequest.*
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController
import com.therekrab.autopilot.APConstraints
import com.therekrab.autopilot.APProfile
import com.therekrab.autopilot.APTarget
import com.therekrab.autopilot.Autopilot
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.LinearAccelerationUnit
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team2471.off2025.util.*
import frc.team2471.off2025.util.logged.LoggedTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.min

abstract class SwerveDriveSubsystem(
    driveConstants: SwerveDrivetrainConstants,
    vararg moduleConstants: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
): SwerveDrivetrain<LoggedTalonFX, LoggedTalonFX, CANcoder>(
    { deviceId: Int, canbus: String? -> LoggedTalonFX(deviceId, canbus) },
    { deviceId: Int, canbus: String? -> LoggedTalonFX(deviceId, canbus) },
    { deviceId: Int, canbus: String? -> CANcoder(deviceId, canbus) },
    driveConstants,
    *moduleConstants
), Subsystem {

    abstract fun getJoystickPercentageSpeeds(): ChassisSpeeds

    abstract val autoPilot: Autopilot

    abstract val pathXController: PIDController //= PIDController(7.0, 0.0, 0.0)
    abstract val pathYController: PIDController //= PIDController(7.0, 0.0, 0.0)
    abstract val pathThetaController: PIDController //= PIDController(7.0, 0.0, 0.0)

    abstract val autoDriveToPointController: PIDController //= PIDController(3.0, 0.0, 0.1)
    abstract val teleopDriveToPointController: PIDController //= PIDController(3.0, 0.0, 0.1)

    abstract val driveAtAnglePIDController: PhoenixPIDController

    var savedState: SwerveDriveState = stateCopy

    @get:AutoLogOutput(key = "Drive/Pose")
    abstract var pose: Pose2d // Abstract to allow for other pose sources (cameras) to also reset when this gets set.

    abstract var heading: Rotation2d

    @get:AutoLogOutput(key = "Drive/Speeds")
    val speeds: ChassisSpeeds
        get() = savedState.Speeds.robotToFieldCentric(pose.rotation)

    @get:AutoLogOutput(key = "Drive/Velocity")
    val velocity: LinearVelocity
        get() {
            val speed = savedState.Speeds
            return hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond).metersPerSecond
        }
    private var prevVelocity = velocity

    @get:AutoLogOutput(key = "Drive/Acceleration")
    var acceleration: LinearAcceleration = 0.0.feetPerSecondPerSecond
        private set

    @get:AutoLogOutput(key = "Drive/Jerk")
    var jerk: Velocity<LinearAccelerationUnit> = 0.0.feetPerSecondPerSecond.perSecond
        private set

    private var prevTime = -0.02

    @get:AutoLogOutput(key = "Drive/ModuleStates")
    val moduleStates: Array<SwerveModuleState>
        get() = savedState.ModuleStates

    @get:AutoLogOutput(key = "Drive/ModuleTargets")
    val moduleTargets: Array<SwerveModuleState>
        get() = savedState.ModuleTargets

    @get:AutoLogOutput(key = "Drive/ModulePositions")
    val modulePositions: Array<SwerveModulePosition>
        get() = savedState.ModulePositions

    @get:AutoLogOutput(key = "Drive/RawHeading")
    val rawHeading: Rotation2d
        get() = savedState.RawHeading.wrap()

    @get:AutoLogOutput(key = "Drive/StateTimestamp")
    val stateTimestamp: Double
        get() = savedState.Timestamp

    @get:AutoLogOutput(key = "Drive/OdometryPeriod")
    val odometryPeriod: Double
        get() = savedState.OdometryPeriod

    @get:AutoLogOutput(key = "Drive/Daqs/SuccessfulDaqs")
    val successfulDaqs: Int
        get() = savedState.SuccessfulDaqs

    @get:AutoLogOutput(key = "Drive/Daqs/FailedDaqs")
    val failedDaqs: Int
        get() = savedState.FailedDaqs

    private val gyro: Pigeon2
        get() = pigeon2

    @get:AutoLogOutput(key = "Drive/Gyro/Yaw")
    val gyroYaw: Angle
        get() = BaseStatusSignal.getLatencyCompensatedValueAsDouble(gyro.yaw, gyro.angularVelocityZWorld).degrees.wrap()

    @get:AutoLogOutput(key = "Drive/Gyro/Pitch")
    val gyroPitch: Angle
        get() = BaseStatusSignal.getLatencyCompensatedValueAsDouble(gyro.pitch, gyro.angularVelocityXWorld).degrees.wrap()

    @get:AutoLogOutput(key = "Drive/Gyro/Roll")
    val gyroRoll: Angle
        get() = BaseStatusSignal.getLatencyCompensatedValueAsDouble(gyro.roll, gyro.angularVelocityYWorld).degrees.wrap()

    @get:AutoLogOutput(key = "Drive/Gyro/YawRate")
    val gyroYawRate: AngularVelocity
        get() = gyro.angularVelocityZWorld.valueAsDouble.degreesPerSecond
    @get:AutoLogOutput(key = "Drive/Gyro/PitchRate")
    val gyroPitchRate: AngularVelocity
        get() = gyro.angularVelocityXWorld.valueAsDouble.degreesPerSecond
    @get:AutoLogOutput(key = "Drive/Gyro/RollRate")
    val gyroRollRate: AngularVelocity
        get() = gyro.angularVelocityYWorld.valueAsDouble.degreesPerSecond

    @get:AutoLogOutput(key = "Drive/Gyro/AccelerationX")
    val gyroAccelerationX: LinearAcceleration
        get() = (gyro.accelerationX.valueAsDouble - gyro.gravityVectorX.valueAsDouble).Gs
    @get:AutoLogOutput(key = "Drive/Gyro/AccelerationY")
    val gyroAccelerationY: LinearAcceleration
        get() = (gyro.accelerationY.valueAsDouble - gyro.gravityVectorY.valueAsDouble).Gs


    /** Returns an array of module translations. */
    val moduleTranslationsMeters = moduleConstants.map { Translation2d(it.LocationX.meters, it.LocationY.meters) }.toTypedArray()

    val driveBaseRadius = moduleTranslationsMeters.maxOf { it.norm }.meters

    /** Returns the maximum linear speed */
    val maxSpeed: LinearVelocity = moduleConstants.first().SpeedAt12Volts.metersPerSecond

    /** Returns the maximum angular speed in radians per sec.  */
    val maxAngularSpeedRadPerSec: AngularVelocity = (maxSpeed.asMetersPerSecond / driveBaseRadius.asMeters).radiansPerSecond


    private val driveAtAngleRequest = FieldCentricFacingAngle()


    private val gyroDisconnectedAlert = Alert("Gyro Disconnected", Alert.AlertType.kError)
    private val driveDisconnectAlerts = Array(moduleConstants.size) { Alert("Module $it Drive Motor Disconnected", Alert.AlertType.kError) }
    private val steerDisconnectAlerts = Array(moduleConstants.size) { Alert("Module $it Steer Motor Disconnected", Alert.AlertType.kError) }
    private val encoderDisconnectAlerts = Array(moduleConstants.size) { Alert("Module $it Encoder Disconnected", Alert.AlertType.kError) }
    private var moduleErrorIndex = 0

    init {
        //Register the subsystem into the CommandScheduler so periodic methods can be called.
        CommandScheduler.getInstance().registerSubsystem(this)
    }

    fun finalInitialization() {
        driveAtAngleRequest.apply {
            HeadingController = driveAtAnglePIDController.apply {
                enableContinuousInput(-Math.PI, Math.PI)
            }
            DriveRequestType = SwerveModule.DriveRequestType.Velocity
        }
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI)
    }

    override fun periodic() {
        LoopLogger.record("b4 super drive")
        savedState = stateCopy //This line takes a long time when data acquisitions fail
        LoopLogger.record("drive state set")
        gyroDisconnectedAlert.set(!gyro.isConnected)
        LoopLogger.record("b4 gyro connect")

        // Check if a part of any modules have been disconnected. Save on cycle time by only checking one module every loop.
        val module = modules[moduleErrorIndex]
        driveDisconnectAlerts[moduleErrorIndex].set(!module.driveMotor.isConnected)
        steerDisconnectAlerts[moduleErrorIndex].set(!module.steerMotor.isConnected)
        encoderDisconnectAlerts[moduleErrorIndex].set(!module.encoder.isConnected)
        moduleErrorIndex = (moduleErrorIndex + 1) % modules.size

        LoopLogger.record("drive modules")

        val currTime = Timer.getFPGATimestamp()
        LoopLogger.record("drive timetime")
        val currVelocity = velocity
        LoopLogger.record("drive getVelocity")
        val prevAcceleration = acceleration
        LoopLogger.record("drive setPrevAccel")
        val deltaTime = currTime - prevTime

        LoopLogger.record("drive get time")
        acceleration = ((currVelocity - prevVelocity) / deltaTime).perSecond
        jerk = ((acceleration - prevAcceleration) / deltaTime).perSecond

        prevVelocity = currVelocity
        prevTime = currTime
    }

    /**
     * Runs the drive at the desired velocity. Field centric
     * @param speeds Speeds in meters/sec
     */
    fun driveVelocity(speeds: ChassisSpeeds) {
        Logger.recordOutput("Drive/Wanted ChassisSpeeds", speeds.fieldToRobotCentric(heading))
        setControl(
            ApplyFieldSpeeds().apply{
                Speeds = speeds
                DriveRequestType = SwerveModule.DriveRequestType.Velocity
            }
        )
    }

    /**
     * Runs the drive at the desired voltage. Field centric
     * @param speedsInVolts Speeds in volts
     */
    fun driveVoltage(speedsInVolts: ChassisSpeeds) {
        setControl(
            ApplyFieldSpeeds().apply {
                Speeds = speedsInVolts
                DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage
            }
        )
    }

    /**
     * Returns the wanted chassis speeds from the joystick.
     * @see getJoystickPercentageSpeeds
     * @see maxSpeed
     */
    fun getChassisSpeedsFromJoystick(): ChassisSpeeds = getJoystickPercentageSpeeds().apply {
        vxMetersPerSecond *= maxSpeed.asMetersPerSecond
        vyMetersPerSecond *= maxSpeed.asMetersPerSecond
        omegaRadiansPerSecond *= maxAngularSpeedRadPerSec.asRadiansPerSecond
    }


    // All of these driveAtAngle function variations exist to make syntax good when calling the function
    fun driveAtAngle(angle: Rotation2d): Command = driveAtAngle { angle }
    fun driveAtAngle(angle: () -> Rotation2d): Command = driveAtAngle(angle) { getChassisSpeedsFromJoystick().translation }
    fun driveAtAngle(
        angle: () -> Rotation2d,
        translation: () -> Translation2d = { getChassisSpeedsFromJoystick().translation }
    ): Command = run {
        driveAtAngle(angle(), translation())
    }
    /**
     * Uses the [driveAtAnglePIDController] to drive the robot with a specified angle and translation.
     */
    fun driveAtAngle(angle: Rotation2d, translation: Translation2d) {
        setControl(
            driveAtAngleRequest.apply {
                VelocityX = translation.x
                VelocityY = translation.y
                TargetDirection = angle
            }
        )
    }

    /**
     * Set the swerve drive module states to point inward on the robot in an "X" fashion.
     */
    fun xPose() = setControl(SwerveDriveBrake())

    /**
     * Applies a 0v output to the drivetrain.
     */
    fun stop() = driveVoltage(ChassisSpeeds())




    override fun resetPose(pose2d: Pose2d) {
        resetTranslation(pose2d.translation)
        heading = pose2d.rotation
    }

    override fun resetRotation(rotation: Rotation2d) {
        super.resetRotation(rotation)
//        gyro.setYaw(rotation.measure)
    }

    /**
     * Set all the drive and steer motors to brake mode.
     */
    fun brakeMode() {
        modules.forEach {
            it.steerMotor.brakeMode()
            it.driveMotor.brakeMode()
        }
    }

    /**
     * Set all the drive and steer motors to coast mode.
     */
    fun coastMode() {
        modules.forEach {
            it.steerMotor.coastMode()
            it.driveMotor.coastMode()
        }
    }

    /**
     * Set the module offsets to the current position of the module.
     */
    fun setAngleOffsets(): Command = runOnce {
        val offsets = modules.map { it.encoder.setCANCoderAngle(0.0.degrees) }
        offsets.forEachIndexed { i, offset ->
            Preferences.setDouble("Module $i Offset", offset.asDegrees)
        }
    }


    /**
     * Drives the robot using the joystick.
     */
    fun joystickDrive(): Command {
        return Commands.run({
            LoopLogger.record("b4 joystickDrive")
            // Get linear velocity
            val chassisSpeeds = getChassisSpeedsFromJoystick().apply {
                if (isBlueAlliance) {
                    vxMetersPerSecond *= -1.0
                    vyMetersPerSecond *= -1.0
                }
            }

            //send it
            driveVelocity(chassisSpeeds)

            LoopLogger.record("joystickDrive")
        },
            this
        )
    }

    /**
     * Drives the robot to a [wantedPose]. Uses the [autoDriveToPointController] or [teleopDriveToPointController]
     *
     * @param wantedPose The pose to drive to
     * @param poseSupplier A function that returns the pose of the robot. The default value is the swerve odometry.
     * @param exitSupplier A function that returns true if the command should abort. The default value ends when the robot is within 0.75 meters of the target.
     * @param maxVelocity The maximum velocity of the robot. The default value is [maxSpeed] from constants.
     */
    fun driveToPoint(
        wantedPose: Pose2d,
        poseSupplier: () -> Pose2d = { pose },
        exitSupplier: (Distance, Angle) -> Boolean = { error, headingError -> error < 0.75.inches && headingError < 1.0.degrees },
        maxVelocity: LinearVelocity = maxSpeed
    ): Command {
        var distanceToPose: Double = Double.POSITIVE_INFINITY

        Logger.recordOutput("Drive/DriveToPoint/Point", wantedPose)


        return run {
            val translationToPose = wantedPose.translation.minus(poseSupplier().translation)
            distanceToPose = translationToPose.norm
            val pidController = if (DriverStation.isAutonomous()) autoDriveToPointController else teleopDriveToPointController
            val velocityOutput = min(abs(pidController.calculate(distanceToPose, 0.0)), maxVelocity.asMetersPerSecond)
            val wantedVelocity = translationToPose.normalize() * velocityOutput
            driveAtAngle(wantedPose.rotation, wantedVelocity)
        }.until {
            val distanceError = distanceToPose.meters
            val headingError = (wantedPose.rotation - poseSupplier().rotation).measure.absoluteValue()

            Logger.recordOutput("Drive/DriveToPoint/DistanceError", distanceError)
            Logger.recordOutput("Drive/DriveToPoint/HeadingError", headingError)

            exitSupplier(distanceError, headingError)
        }.finallyRun {
            stop()
            Logger.recordOutput("Drive/DriveToPoint/Point", Pose2d())
        }.withName("DriveToPoint")
    }

    /**
     * Drives the robot to a [wantedPose] using Autopilot. Uses [autoPilot] to control the robot.
     *
     * Finishes when the [Autopilot.atTarget] method returns true.
     *
     * @param wantedPose The position to drive to
     * @param poseSupplier A function that returns the pose of the robot. The default value is the swerve odometry.
     * @param entryAngle The angle [Autopilot] will try to enter the wantedPose at. The default value is null.
     */
    fun driveToAutopilotPoint(
        wantedPose: Pose2d,
        poseSupplier: () -> Pose2d = { pose },
        entryAngle: Angle? = null
    ): Command {
        val target = if (entryAngle != null) {
            APTarget(wantedPose).withEntryAngle(entryAngle.asRotation2d)
        } else {
            APTarget(wantedPose)
        }
        Logger.recordOutput("Drive/AutoPilot/Target", wantedPose)
        return run {
            val output = autoPilot.calculate(poseSupplier(), speeds.translation, target)
            val velocity = Translation2d(output.vx.asMetersPerSecond, output.vy.asMetersPerSecond)
            Logger.recordOutput("Drive/AutoPilot/Velocity", velocity.norm)

//            println("output ${velocity.norm}")
            driveAtAngle(output.targetAngle(), velocity)
        }.until {
            autoPilot.atTarget(poseSupplier(), target)
        }.finallyRun {
            stop()
            Logger.recordOutput("Drive/AutoPilot/Target", Pose2d())
        }
    }


    /**
     * Drives the robot to the closest point along a line but also lets the driver control the robot along it. Uses the [autoDriveToPointController] or [teleopDriveToPointController]
     *
     * @param pointOne The first line endpoint.
     * @param pointTwo The second line endpoint.
     * @param heading The wanted heading of the robot to align to. The default value doesn't restrict the heading and allows joystick rotation control.
     * @param poseSupplier A function that returns the pose of the robot. The default value is the swerve odometry.
     * @param maxVelocity The maximum velocity of the robot. The default value is [maxSpeed] from constants.
     *
     * @see driveToPoint
     */
    fun joystickDriveAlongLine(
        pointOne: Translation2d,
        pointTwo: Translation2d,
        heading: Rotation2d? = null,
        poseSupplier: () -> Pose2d = { pose },
        lineTolerance: Distance = 0.5.inches,
        maxVelocity: LinearVelocity = maxSpeed
    ): Command {
        val lineAngle = (pointTwo - pointOne).angle

        return run {
            val currentPose = poseSupplier()
            val linePoint = findClosestPointOnLine(pointOne, pointTwo, currentPose.translation)
            val translationToPose = linePoint.minus(currentPose.translation)
            val distanceToPose = translationToPose.norm.deadband(lineTolerance.asMeters)
            val driveToPointPower = min(abs(teleopDriveToPointController.calculate(distanceToPose, 0.0)), maxVelocity.asMetersPerSecond)

            val driveToPointVelocity = translationToPose.normalize() * driveToPointPower
            val teleopChassisSpeeds = getChassisSpeedsFromJoystick().apply {
                //Limit joystick speeds to be along the line
                val modifiedTranslation = translation.rotateBy(-lineAngle)
                val lineCentricTranslation = Translation2d(modifiedTranslation.x, 0.0).rotateBy(lineAngle)
                vxMetersPerSecond = lineCentricTranslation.x
                vyMetersPerSecond = lineCentricTranslation.y
            }

            Logger.recordOutput("Drive/AlongLine/line", *arrayOf(pointOne, pointTwo))
            Logger.recordOutput("Drive/AlongLine/closestPoint", linePoint.toPose2d())
            Logger.recordOutput("Drive/AlongLine/translation2Pose", translationToPose.toPose2d())


            if (heading == null) {
                val driveToPointChassisSpeeds = ChassisSpeeds(driveToPointVelocity.x, driveToPointVelocity.y, 0.0)
                val wantedChassisSpeeds = driveToPointChassisSpeeds + teleopChassisSpeeds
                driveVelocity(wantedChassisSpeeds)
            } else {
                val wantedVelocity = teleopChassisSpeeds.translation + driveToPointVelocity
                driveAtAngle(heading, wantedVelocity)
            }

        }.finallyRun {
            Logger.recordOutput("Drive/AlongLine/line", *arrayOf<Translation2d>())
            Logger.recordOutput("Drive/AlongLine/closestPoint", Pose2d())
        }
    }


    /**
     * Drives the robot to the closest point along a line specified by [pointOne] and [pointTwo]
     * @see driveToPoint
     */
    fun driveToLine(
        pointOne: Translation2d,
        pointTwo: Translation2d,
        heading: Rotation2d,
        poseSupplier: () -> Pose2d = { pose },
        exitSupplier: ((Distance, Angle) -> Boolean)? = null,
        maxVelocity: LinearVelocity = maxSpeed
    ): Command {
        var closestPoseOnLine: Pose2d? = null

        return sequenceCommand(
            runOnce {
                println("driveToPointOnLine")

                closestPoseOnLine = findClosestPointOnLine(pointOne, pointTwo, poseSupplier().translation).toPose2d(heading)

                Logger.recordOutput("Drive/ToPointOnLine/Points", *arrayOf(pointOne, pointTwo))
                Logger.recordOutput("Drive/ToPointOnLine/ClosestPose", closestPoseOnLine)
            },
            Commands.either(
                defer { driveToPoint(closestPoseOnLine!!, poseSupplier, maxVelocity = maxVelocity) },
                defer { driveToPoint(closestPoseOnLine!!, poseSupplier,exitSupplier!!, maxVelocity) },
                { exitSupplier == null })
        ).finallyRun {
            Logger.recordOutput("Drive/ToPointOnLine/Points", *arrayOf<Translation2d>())
            Logger.recordOutput("Drive/ToPointOnLine/ClosestPose", *arrayOf<Pose2d>())
        }
    }

    /**
     * Drives the robot along a path from Choreo. Uses the [pathXController], [pathYController], and [pathThetaController] to control the robot.
     *
     * @param path The path to drive along
     * @param poseSupplier A function that returns the pose of the robot. The default value is the swerve odometry.
     * @param resetOdometry Whether to reset the odometry to the start of the path. The default value is false.
     * @param exitSupplier A function that returns true if the command should abort. The default value ends when the path duration finishes.
     */
    fun driveAlongChoreoPath(
        path: Trajectory<SwerveSample>,
        poseSupplier: () -> Pose2d = { pose },
        resetOdometry: Boolean = false,
        exitSupplier: (Double) -> Boolean = { it >= 1.0 }
    ): Command {
        val totalTime = path.totalTime
        var t = 0.0
        val timer = Timer()

        return run {
            val currentPose = poseSupplier()
            t = min(timer.get(), totalTime)
            val sample = path.sampleAt(t, false).get()
            val wantedPose = sample.pose
            val wantedSpeeds = sample.chassisSpeeds
            val moduleForcesX = sample.moduleForcesX()
            val moduleForcesY = sample.moduleForcesY()

            Logger.recordOutput("Drive/Path/Time", t)
            Logger.recordOutput("Drive/Path/Pose", wantedPose)
            Logger.recordOutput("Drive/Path/Speeds", wantedSpeeds)
            Logger.recordOutput("Drive/Path/Module Forces X", moduleForcesX)
            Logger.recordOutput("Drive/Path/Module Forces Y", moduleForcesY)

            wantedSpeeds.apply {
                vxMetersPerSecond += pathXController.calculate(currentPose.x, wantedPose.x)
                vyMetersPerSecond += pathYController.calculate(currentPose.y, wantedPose.y)
                omegaRadiansPerSecond += pathThetaController.calculate(currentPose.rotation.radians, sample.heading)
            }
            setControl(
                ApplyFieldSpeeds().apply {
                    Speeds = wantedSpeeds
                    WheelForceFeedforwardsX = moduleForcesX
                    WheelForceFeedforwardsY = moduleForcesY
                    DriveRequestType = SwerveModule.DriveRequestType.Velocity
                }
            )
        }.beforeRun {
            if (resetOdometry) {
                pose = path.getInitialPose(false).get()
            }

            println("Running DriveAlongChoreoPath")

            Logger.recordOutput("Drive/Path/Name", path.name())
            Logger.recordOutput("Drive/Path/TotalTime", totalTime)

            t = 0.0

            timer.restart()
        }.until {
            val p = t / totalTime
            Logger.recordOutput("Drive/Path/Done %", p)
            exitSupplier(p)
        }.finallyRun {
            // Tell drivetrain to apply no output
            stop()

            println("Finished driveAlongChoreoPath at ${(t / totalTime * 100.0).round(1)}% done")
            // Publish empty data to show that the path is done
            Logger.recordOutput("Drive/Path/Pose", Pose2d())
        }.withName("DriveAlongChoreoPath")
    }



    /**
     * Simple constructor for creating an Autopilot object.
     *
     * @param maxVelocity The maximum velocity Autopilot will allow. meters/sec
     * @param maxAcceleration The maximum acceleration Autopilot will allow. meters/sec^2
     * @param maxJerk The maximum jerk Autopilot will allow. meters/sec^3
     * @param xyTolerance The xy translation tolerance for the robot to be at the target position, effects [Autopilot.atTarget]. Meters
     * @param thetaTolerance The theta rotation tolerance for the robot to be at the target position, effects [Autopilot.atTarget]. Radians
     * @param beelineRadius The beeline radius is a distance where, under that range, an entry angle is no longer respected. Default value 8 cm
     *
     * @see Autopilot
     * @see APConstraints
     * @see APProfile
     */
    fun createAPObject(maxVelocity: Double, maxAcceleration: Double, maxJerk: Double, xyTolerance: Distance, thetaTolerance: Angle, beelineRadius: Distance = 8.0.centimeters): Autopilot {
        return Autopilot(APProfile(APConstraints(maxVelocity, maxAcceleration, maxJerk))
            .withErrorXY(xyTolerance).withErrorTheta(thetaTolerance).withBeelineRadius(beelineRadius)
        )
    }

    // sysID

    private val translationSysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            7.0.volts,
            5.0.seconds
        ) { state: SysIdRoutineLog.State ->
            SignalLogger.writeString("SysIdTranslation_State", state.toString())
            Logger.recordOutput("SysIdTranslation_State", state.toString())
          },
        Mechanism({ output: Voltage -> setControl(SysIdSwerveTranslation().withVolts(output))}, null, this)
    )
    //used to find driveAtAngleRequest PID
    private val rotationSysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            Units.Volts.of(Math.PI / 6).per(Units.Second),
            Math.PI.volts,
            5.0.seconds
        ) { state: SysIdRoutineLog.State ->
            SignalLogger.writeString("SysIdRotation_State", state.toString())
            Logger.recordOutput("SysIdRotation_State", state.toString())
          },
        Mechanism({ output: Voltage ->
            /* output is actually radians per second, but SysId only supports "volts" */
            setControl(SysIdSwerveRotation().withRotationalRate(output.asVolts))
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.asVolts)
            Logger.recordOutput("Rotational_Rate", output.asVolts)
        }, null, this)
    )
    private val steerSysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            7.0.volts,
            null
        ) { state: SysIdRoutineLog.State ->
            SignalLogger.writeString("SysIdSteer_State", state.toString())
            Logger.recordOutput("SysIdSteer_State", state.toString())
          },
        Mechanism({ volts: Voltage? -> setControl(SysIdSwerveSteerGains().withVolts(volts)) }, null, this)
    )

    fun sysIDTranslationDynamic(direction: SysIdRoutine.Direction): Command = translationSysIdRoutine.dynamic(direction).finallyWait(1.0)
    fun sysIDTranslationQuasistatic(direction: SysIdRoutine.Direction): Command = translationSysIdRoutine.quasistatic(direction).finallyWait(1.0)
    fun sysIDRotationDynamic(direction: SysIdRoutine.Direction): Command = rotationSysIdRoutine.dynamic(direction).finallyWait(1.0)
    fun sysIDRotationQuasistatic(direction: SysIdRoutine.Direction): Command = rotationSysIdRoutine.quasistatic(direction).finallyWait(1.0)
    fun sysIDSteerDynamic(direction: SysIdRoutine.Direction): Command = steerSysIdRoutine.dynamic(direction).finallyWait(1.0)
    fun sysIDSteerQuasistatic(direction: SysIdRoutine.Direction): Command = steerSysIdRoutine.quasistatic(direction).finallyWait(1.0)

    fun sysIDTranslationAll() = sequenceCommand(
        // Quasistatic forward and reverse
        sysIDTranslationQuasistatic(SysIdRoutine.Direction.kForward), sysIDTranslationQuasistatic(SysIdRoutine.Direction.kReverse),
        // Dynamic forward and reverse
        sysIDTranslationDynamic(SysIdRoutine.Direction.kForward), sysIDTranslationDynamic(SysIdRoutine.Direction.kReverse)
    )
    fun sysIDRotationAll() = sequenceCommand(
        // Quasistatic forward and reverse
        sysIDRotationQuasistatic(SysIdRoutine.Direction.kForward), sysIDRotationQuasistatic(SysIdRoutine.Direction.kReverse),
        // Dynamic forward and reverse
        sysIDRotationDynamic(SysIdRoutine.Direction.kForward), sysIDRotationDynamic(SysIdRoutine.Direction.kReverse),
    )
    fun sysIDSteerAll() = sequenceCommand(
        // Quasistatic forward and reverse
        sysIDSteerQuasistatic(SysIdRoutine.Direction.kForward), sysIDSteerQuasistatic(SysIdRoutine.Direction.kReverse),
        // Dynamic forward and reverse
        sysIDSteerDynamic(SysIdRoutine.Direction.kForward), sysIDSteerDynamic(SysIdRoutine.Direction.kReverse),
    )


    override fun updateSimState(dtSeconds: Double, supplyVoltage: Double) {
        if (isSim) {
            super.updateSimState(dtSeconds, supplyVoltage)
        } else {
            DriverStation.reportError("DriveIOCTRE.updateSim() called while robot is real", true)
            throw Error("DriveIOCTRE.updateSim() called while robot is real")
        }
    }
}
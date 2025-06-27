// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.team2471.off2025.subsystems.drive

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team2471.off2025.OI
import frc.team2471.off2025.Robot
import frc.team2471.off2025.generated.TunerConstants
import frc.team2471.off2025.subsystems.drive.gyro.GyroIO
import frc.team2471.off2025.subsystems.drive.gyro.GyroIOInputsAutoLogged
import frc.team2471.off2025.subsystems.drive.gyro.GyroIONavX
import frc.team2471.off2025.subsystems.drive.gyro.GyroIOPigeon2
import frc.team2471.off2025.util.*
import frc.team2471.off2025.util.swerve.SwerveSetpointGenerator
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.roundToInt
import kotlin.math.sin

object Drive : SubsystemBase("Drive") {
    private val modules =  arrayOf(
        Module(0, TunerConstants.FrontLeft),
        Module(1, TunerConstants.FrontRight),
        Module(2, TunerConstants.BackLeft),
        Module(3, TunerConstants.BackRight)
    ) // FL, FR, BL, BR

    private val gyroInputs: GyroIOInputsAutoLogged = GyroIOInputsAutoLogged()
    private val gyroIO: GyroIO = when (robotMode) {
        RobotMode.REAL -> if (Robot.isCompBot) GyroIONavX() else GyroIOPigeon2()
        RobotMode.SIM -> GyroIOPigeon2()
        else -> object : GyroIO {}
    }

    private var rawGyroRotation = Rotation2d()

    private val kinematics = SwerveDriveKinematics(*TunerConstants.moduleTranslations)
    private val lastModulePositions = Array(4) { SwerveModulePosition() }// For delta tracking
    private var lastModulePositionsArc = Array(4) { SwerveModulePosition() }

    private val poseEstimator = SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d())

    /** Returns the maximum linear speed in meters per sec.  */
    val maxLinearSpeedMetersPerSec: Double = TunerConstants.kSpeedAt12Volts.asMetersPerSecond
    /** Returns the maximum angular speed in radians per sec.  */
    val maxAngularSpeedRadPerSec: Double = this.maxLinearSpeedMetersPerSec / TunerConstants.DRIVE_BASE_RADIUS

    // PathPlanner config constants
    private val ROBOT_MASS_KG = 135.0.pounds.asKilograms
    private const val ROBOT_MOI = 6.883 //3KG m^2
    private const val WHEEL_COF = 1.2

    private val pathPlannerConfig: RobotConfig = RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOI,
        ModuleConfig(
            TunerConstants.FrontLeft.WheelRadius,
            TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond),
            WHEEL_COF,
            DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
            TunerConstants.FrontLeft.SlipCurrent,
            1
        ),
        *TunerConstants.moduleTranslations
    )

    private val sysId: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            0.5.voltsPerSecond,
            5.0.volts,
            5.0.seconds
        ) { state: SysIdRoutineLog.State? -> Logger.recordOutput("Drive/SysIdState", state.toString()) },
        Mechanism({ voltage: Voltage? -> runCharacterization(voltage!!.`in`(Units.Volts)) }, {
            SysIdRoutineLog("hi").motor("drive")
        }, this)
    )

    private val gyroDisconnectedAlert = Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError)

    val setpointGenerator = SwerveSetpointGenerator(kinematics, TunerConstants.moduleTranslations,
        SwerveSetpointGenerator.ModuleLimits(
            4.75,//TunerConstants.kSpeedAt12Volts.asMetersPerSecond,
            22.0,
            60.0.rotationsPerSecond.asRadiansPerSecond
        )
    )
    var prevOptimizedSetpoint = SwerveSetpointGenerator.SwerveSetpoint(ChassisSpeeds(), Array(4) { SwerveModuleState()})

    @get:AutoLogOutput(key = "SwerveStates/Measured")
    private val moduleStates: Array<SwerveModuleState>
        /** Returns the module states (turn angles and drive velocities) for all the modules.  */
        get() = Array(4) { modules[it].state }

    private val modulePositions: Array<SwerveModulePosition>
        /** Returns the module positions (turn angles and drive positions) for all the modules.  */
        get() = Array(4) { modules[it].position}

    @get:AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    val chassisSpeeds: ChassisSpeeds
        /** Returns the measured chassis speeds of the robot.  */
        get() = kinematics.toChassisSpeedsK(this.moduleStates)

    val wheelRadiusCharacterizationPositions: DoubleArray
        /** Returns the position of each module in radians.  */
        get() = DoubleArray(4) { modules[it].wheelRadiusCharacterizationPosition }

    val fFCharacterizationVelocity: Double
        /** Returns the average velocity of the modules in rotations/sec (Phoenix native units).  */
        get() = modules.sumOf { it.fFCharacterizationVelocity / 4.0 }

    @get:AutoLogOutput(key = "Odometry/frc.team2471.off2025.Robot")
    var pose: Pose2d
        /** Returns the current odometry pose.  */
        get() = poseEstimator.estimatedPosition
        /** Resets the current odometry pose.  */
        set(pose) {
            poseEstimator.resetPosition(rawGyroRotation, this.modulePositions, pose)
        }

    @get:AutoLogOutput(key = "Odometry/arcPose")
    var arcPose: Pose2d = Pose2d()

    val rotation: Rotation2d
        /** Returns the current odometry rotation.  */
        get() = this.pose.rotation

    init {
        // Start odometry thread
        OdometrySignalThread.start()

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            { this.pose },
            { pose: Pose2d? ->
                if (pose != null) {
                    this.pose = pose
                }
            },
            { this.chassisSpeeds },
            { speeds: ChassisSpeeds? -> this.driveVelocity(speeds!!) },
            PPHolonomicDriveController(PIDConstants(5.0, 0.0, 0.0), PIDConstants(5.0, 0.0, 0.0)),
            pathPlannerConfig,
            { isRedAlliance },
            this
        )
        Pathfinding.setPathfinder(LocalADStarAK())
        PathPlannerLogging.setLogActivePathCallback { activePath: MutableList<Pose2d> ->
            Logger.recordOutput("Odometry/Trajectory", *activePath.toTypedArray<Pose2d>())
        }
        PathPlannerLogging.setLogTargetPoseCallback { targetPose: Pose2d? ->
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
        }

        val missingPrefsModules = arrayListOf<Int>()
        val conflictingOffsets = arrayListOf<Int>()
        modules.filter { it.isEncoderConnected }.forEachIndexed { i, module ->
            val prefsOffset = Preferences.getDouble("Module $i Offset", Double.NaN).degrees
            val encoderOffset = module.cancoderOffset
            if (prefsOffset.asDegrees.isNaN()) {
                //couldn't find prefs
                println("module $i has missing Preferences, setting to encoders Offset: ${encoderOffset.asDegrees.roundToInt()}")
                Preferences.setDouble("Module $i Offset", encoderOffset.asDegrees)
                missingPrefsModules.add(i)
            } else if (prefsOffset.asDegrees.roundToInt() != encoderOffset.asDegrees.roundToInt()) {
                //offsets are different, default to prefs
                println("module $i has conflicting offsets. prefsOffset: ${prefsOffset.asDegrees.round(2)} encoders Offset: ${encoderOffset.asDegrees.round(2)}")
                module.setCANCoderOffset(prefsOffset)
                conflictingOffsets.add(i)
            }
        }

        missingPrefsModules.forEach { Alert("Module $it had missing prefs", AlertType.kWarning).set(true) }
        conflictingOffsets.forEach { Alert("Module $it had conflicting offsets", AlertType.kWarning).set(true) }
    }

    override fun periodic() {
        OdometrySignalThread.odometryLock.lock() // Prevents updates while reading data
        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("Drive/Gyro", gyroInputs)
        modules.forEach { it.periodic() }
        OdometrySignalThread.odometryLock.unlock()

        if (DriverStation.isDisabled()) {
            // Stop moving when disabled
            driveVelocity(ChassisSpeeds())
            // Log empty setpoint states when disabled
            Logger.recordOutput("SwerveStates/Setpoints", *arrayOf<SwerveModuleState>())
            Logger.recordOutput("SwerveStates/SetpointsOptimized", *arrayOf<SwerveModuleState>())
        }

        // Update odometry
        val sampleTimestamps = modules.first().odometryTimestamps // All signals are sampled together
        for (i in sampleTimestamps.indices) {
            // Read wheel positions and deltas from each module
            val modulePositions = arrayOfNulls<SwerveModulePosition>(4)
            val moduleDeltas = arrayOfNulls<SwerveModulePosition>(4)
            for (moduleIndex in 0..3) {
                modulePositions[moduleIndex] = modules[moduleIndex].odometryPositions[i]
                moduleDeltas[moduleIndex] = SwerveModulePosition(
                    modulePositions[moduleIndex]!!.distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex]!!.angle
                )
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex]!!
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i]
            } else {
                // Use the angle delta from the kinematics and module deltas
                val twist = kinematics.toTwist2d(*moduleDeltas)
//                println("twist: $twist")
                rawGyroRotation = rawGyroRotation.plus(Rotation2d(twist.dtheta))
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions)

            updateArcOdometry(modulePositions, if (gyroInputs.connected) gyroInputs.odometryYawPositions[i] else null, sampleTimestamps[i])
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && isReal)
    }

    private fun updateArcOdometry(modulePositions: Array<SwerveModulePosition?>, gyroAngle: Rotation2d?, timestamp: Double) {
        val moduleDeltas = Array(modulePositions.size) { i ->
            val prevAngle = lastModulePositionsArc[i].angle
            val currAngle = modulePositions[i]!!.angle
            val deltaDistanceMeters = (modulePositions[i]!!.distanceMeters - lastModulePositionsArc[i].distanceMeters)
            val deltaAngleRad = currAngle.radians - prevAngle.radians
            lastModulePositionsArc[i] = modulePositions[i]!!

            if (deltaAngleRad == 0.0) {
                SwerveModulePosition(deltaDistanceMeters, currAngle)
            } else {
                val radius = deltaDistanceMeters / deltaAngleRad

                val centerOfArcToPrevPos = Translation2d(
                    radius * cos(prevAngle.radians - Math.PI / 2.0),
                    radius * sin(prevAngle.radians - Math.PI / 2.0)
                )
                val centerOfArcToCurrPos = centerOfArcToPrevPos.rotateBy(deltaAngleRad.radians.asRotation2d())
                val arcDisplacement = centerOfArcToCurrPos - centerOfArcToPrevPos

                SwerveModulePosition(arcDisplacement.norm, Rotation2d(atan2(arcDisplacement.y, arcDisplacement.x)))
            }
        }

        Logger.recordOutput("Odometry/ModuleDeltas", *(moduleDeltas.map { SwerveModuleState(it.distanceMeters, it.angle) }).toTypedArray<SwerveModuleState>())

        val twist = kinematics.toTwist2d(*moduleDeltas)
        val gyroDelta = twist.dtheta.radians.asDegrees

        arcPose = Pose2d(arcPose.exp(twist).translation, gyroAngle ?: (arcPose.rotation.measure.asDegrees + gyroDelta).degrees.asRotation2d())
    }

    fun getChassisSpeedsFromJoystick(): ChassisSpeeds {
        //make joystick pure circle
        val (cx, cy) = OI.unsnapAndDesaturateJoystick(OI.driveTranslationX, OI.driveTranslationY)

        //square drive input
        val mult = hypot(cx, cy).square()
        val (x, y) = Pair(cx * mult, cy * mult)

        val omega = OI.driveRotation.cube()

        val chassisSpeeds = ChassisSpeeds(
            x * maxLinearSpeedMetersPerSec,
            y * maxLinearSpeedMetersPerSec,
            omega * maxAngularSpeedRadPerSec
        )

//        println("linearVelocity = ${chassisSpeeds.translation.norm}")

        return chassisSpeeds
    }

    /**
     * Runs the drive at the desired velocity.
     * @param speeds Speeds in meters/sec
     */
    fun driveVelocity(speeds: ChassisSpeeds) {
        // Calculate module setpoints
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)

        val generatedSetpoints = setpointGenerator.generateSetpoint(
            prevOptimizedSetpoint,
            discreteSpeeds,
            0.02
        )
        prevOptimizedSetpoint = generatedSetpoints

        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts)

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
        Logger.recordOutput("SwerveStates/GeneratedSetpoints", *generatedSetpoints.moduleStates)
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds)
        Logger.recordOutput("SwerveChassisSpeeds/GeneratedSetpoints", generatedSetpoints.chassisSpeeds)

        // Send setpoints to modules
        modules.forEachIndexed { i, module ->
            module.runVelocitySetpoint(setpointStates[i])
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", *setpointStates)
    }

    fun driveVoltage(speedsInVolts: ChassisSpeeds) {
        val discreteSpeeds = ChassisSpeeds.discretize(speedsInVolts, 0.02)
        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)

        modules.forEachIndexed { i, module ->
            module.runVoltageSetpoint(setpointStates[i])
        }
    }


    /** Runs the drive in a straight line with the specified drive output.  */
    fun runCharacterization(output: Double) = modules.forEach { it.runStraight(output) }

    fun stop() = driveVelocity(ChassisSpeeds())

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    fun xPose() {
        kinematics.resetHeadings(*Array<Rotation2d>(4) { TunerConstants.moduleTranslations[it].angle })
        stop()
    }

    fun brakeMode() = modules.forEach { it.brakeMode() }
    fun coastMode() = modules.forEach { it.coastMode() }

    /** Returns a command to run a quasistatic test in the specified direction.  */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command =
        run { runCharacterization(0.0) }.withTimeout(1.0).andThen(sysId.quasistatic(direction))


    /** Returns a command to run a dynamic test in the specified direction.  */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command =
        run { runCharacterization(0.0) }.withTimeout(1.0).andThen(sysId.dynamic(direction))


    fun setAngleOffsets(): Command = runOnce {
        val offsets = modules.map { it.setCANCoderAngle(0.0.degrees) }
        offsets.forEachIndexed { i, offset ->
            Preferences.setDouble("Module $i Offset", offset.asDegrees)
        }
    }

    /** Adds a new timestamped vision measurement.  */
    fun addVisionMeasurement(
        visionRobotPoseMeters: Pose2d?,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3?, N1?>?
    ) {
        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs
        )
    }
}

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
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team2471.off2025.Constants
import frc.team2471.off2025.generated.TunerConstants
import frc.team2471.off2025.subsystems.drive.gyro.GyroIO
import frc.team2471.off2025.subsystems.drive.gyro.GyroIOInputsAutoLogged
import frc.team2471.off2025.subsystems.drive.gyro.GyroIOPigeon2
import frc.team2471.off2025.util.LocalADStarAK
import frc.team2471.off2025.util.degrees
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import kotlin.math.hypot
import kotlin.math.max

object Drive : SubsystemBase() {
    const val ODOMETRY_FREQUENCY: Double = 100.0//if (CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD) 250.0 else 100.0

    private val gyroIO: GyroIO = when (Constants.currentMode) {
        Constants.Mode.REAL -> GyroIOPigeon2()
        else -> object : GyroIO {}
    }
    private val gyroInputs: GyroIOInputsAutoLogged = GyroIOInputsAutoLogged()
    private val modules: Array<Module> = when (Constants.currentMode) {
        Constants.Mode.REAL -> arrayOf(
            Module(ModuleIOTalonFX(TunerConstants.FrontLeft), 0, TunerConstants.FrontLeft),
            Module(ModuleIOTalonFX(TunerConstants.FrontRight), 1, TunerConstants.FrontRight),
            Module(ModuleIOTalonFX(TunerConstants.BackLeft), 2, TunerConstants.BackLeft),
            Module(ModuleIOTalonFX(TunerConstants.BackRight), 3, TunerConstants.BackRight)
        )
        Constants.Mode.SIM -> arrayOf(
            Module(ModuleIOSim(TunerConstants.FrontLeft), 0, TunerConstants.FrontLeft),
            Module(ModuleIOSim(TunerConstants.FrontRight), 1, TunerConstants.FrontRight),
            Module(ModuleIOSim(TunerConstants.BackLeft), 2, TunerConstants.BackLeft),
            Module(ModuleIOSim(TunerConstants.BackRight), 3, TunerConstants.BackRight)
        )
        else -> arrayOf(
            Module(object : ModuleIO {}, 0, TunerConstants.FrontLeft),
            Module(object : ModuleIO {}, 1, TunerConstants.FrontRight),
            Module(object : ModuleIO {}, 2, TunerConstants.BackLeft),
            Module(object : ModuleIO {}, 3, TunerConstants.BackRight)
        )
    } // FL, FR, BL, BR
    private val sysId: SysIdRoutine
    private val gyroDisconnectedAlert = Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError)

    private val kinematics = SwerveDriveKinematics(*moduleTranslations)
    private var rawGyroRotation = Rotation2d()
    private val lastModulePositions: Array<SwerveModulePosition> =  // For delta tracking
        arrayOf<SwerveModulePosition>(
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition()
        )
    private val poseEstimator = SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d())


    private val PP_CONFIG: RobotConfig = RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOI,
        ModuleConfig(
            TunerConstants.FrontLeft.WheelRadius,
            TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond),
            WHEEL_COF,
            DCMotor.getKrakenX60Foc(1)
                .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
            TunerConstants.FrontLeft.SlipCurrent,
            1
        ),
        *moduleTranslations
    )


    init {
        // Start odometry thread
        PhoenixOdometryThread.start()

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            { this.pose },
            { pose: Pose2d? ->
                if (pose != null) {
                    this.pose = pose
                }
            },
            { this.chassisSpeeds },
            { speeds: ChassisSpeeds? -> this.runVelocity(speeds!!) },
            PPHolonomicDriveController(PIDConstants(5.0, 0.0, 0.0), PIDConstants(5.0, 0.0, 0.0)),
            PP_CONFIG,
            { DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red },
            this
        )
        Pathfinding.setPathfinder(LocalADStarAK())
        PathPlannerLogging.setLogActivePathCallback { activePath: MutableList<Pose2d?>? ->
            Logger.recordOutput<Pose2d?>("Odometry/Trajectory", *activePath!!.toTypedArray<Pose2d?>())
        }
        PathPlannerLogging.setLogTargetPoseCallback { targetPose: Pose2d? ->
            Logger.recordOutput<Pose2d?>("Odometry/TrajectorySetpoint", targetPose)
        }

        // Configure SysId
        sysId =
            SysIdRoutine(
                SysIdRoutine.Config(
                    null,
                    null,
                    null
                ) { state: SysIdRoutineLog.State? -> Logger.recordOutput("Drive/SysIdState", state.toString()) },
                Mechanism({ voltage: Voltage? -> runCharacterization(voltage!!.`in`(Units.Volts)) }, null, this)
            )
    }

    override fun periodic() {
        PhoenixOdometryThread.odometryLock.lock() // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("Drive/Gyro", gyroInputs)
        for (module in modules) {
            module.periodic()
        }
        PhoenixOdometryThread.odometryLock.unlock()

        if (DriverStation.isDisabled()) {
            // Stop moving when disabled
            for (module in modules) {
                module.stop()
            }
            // Log empty setpoint states when disabled
            Logger.recordOutput<SwerveModuleState?>("SwerveStates/Setpoints", *arrayOf<SwerveModuleState?>())
            Logger.recordOutput<SwerveModuleState?>("SwerveStates/SetpointsOptimized", *arrayOf<SwerveModuleState?>())
        }

        // Update odometry
        val sampleTimestamps = gyroInputs.odometryYawTimestamps // All signals are sampled together
        for (i in 0..<sampleTimestamps.size) {
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
                rawGyroRotation = rawGyroRotation.plus(Rotation2d(twist.dtheta))
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions)
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Constants.Mode.SIM)
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    fun runVelocity(speeds: ChassisSpeeds) {
        // Calculate module setpoints
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts)

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds)

        // Send setpoints to modules
        for (i in 0..3) {
            modules[i].runSetpoint(setpointStates[i])
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", *setpointStates)
    }

    /** Runs the drive in a straight line with the specified drive output.  */
    fun runCharacterization(output: Double) {
        for (i in 0..3) {
            modules[i].runCharacterization(output)
        }
    }

    /** Stops the drive.  */
    fun stop() {
        runVelocity(ChassisSpeeds())
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    fun xPose() {
        val headings = arrayOfNulls<Rotation2d>(4)
        for (i in 0..3) {
            headings[i] = moduleTranslations[i]!!.angle
        }
        kinematics.resetHeadings(*headings)
        stop()
    }

    /** Returns a command to run a quasistatic test in the specified direction.  */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
        return run { runCharacterization(0.0) }.withTimeout(1.0).andThen(sysId.quasistatic(direction))
    }

    /** Returns a command to run a dynamic test in the specified direction.  */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command {
        return run { runCharacterization(0.0) }.withTimeout(1.0).andThen(sysId.dynamic(direction))
    }

    fun zeroTurnEncoders(): Command = run {
        modules.forEach {
            it.setCANCoderAngle(0.0.degrees)
        }
    }

    @get:AutoLogOutput(key = "SwerveStates/Measured")
    private val moduleStates: Array<SwerveModuleState?>
        /** Returns the module states (turn angles and drive velocities) for all the modules.  */
        get() {
            val states = arrayOfNulls<SwerveModuleState>(4)
            for (i in 0..3) {
                states[i] = modules[i].state
            }
            return states
        }

    private val modulePositions: Array<SwerveModulePosition?>
        /** Returns the module positions (turn angles and drive positions) for all the modules.  */
        get() {
            val states = arrayOfNulls<SwerveModulePosition>(4)
            for (i in 0..3) {
                states[i] = modules[i].position
            }
            return states
        }

    @get:AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private val chassisSpeeds: ChassisSpeeds
        /** Returns the measured chassis speeds of the robot.  */
        get() = kinematics.toChassisSpeeds(*this.moduleStates)

    val wheelRadiusCharacterizationPositions: DoubleArray
        /** Returns the position of each module in radians.  */
        get() {
            val values = DoubleArray(4)
            for (i in 0..3) {
                values[i] = modules[i].wheelRadiusCharacterizationPosition
            }
            return values
        }

    val fFCharacterizationVelocity: Double
        /** Returns the average velocity of the modules in rotations/sec (Phoenix native units).  */
        get() {
            var output = 0.0
            for (i in 0..3) {
                output += modules[i].fFCharacterizationVelocity / 4.0
            }
            return output
        }

    @get:AutoLogOutput(key = "Odometry/Robot")
    var pose: Pose2d
        /** Returns the current odometry pose.  */
        get() = poseEstimator.estimatedPosition
        /** Resets the current odometry pose.  */
        set(pose) {
            poseEstimator.resetPosition(rawGyroRotation, this.modulePositions, pose)
        }

    val rotation: Rotation2d
        /** Returns the current odometry rotation.  */
        get() = this.pose.rotation

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

    val maxLinearSpeedMetersPerSec: Double
        /** Returns the maximum linear speed in meters per sec.  */
        get() = TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond)
    val maxAngularSpeedRadPerSec: Double
        /** Returns the maximum angular speed in radians per sec.  */
        get() = this.maxLinearSpeedMetersPerSec / DRIVE_BASE_RADIUS

    // TunerConstants doesn't include these constants, so they are declared locally
    val DRIVE_BASE_RADIUS: Double = max(
        max(
            hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)
        ),
        max(
            hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        )
    )

    // PathPlanner config constants
    private const val ROBOT_MASS_KG = 74.088
    private const val ROBOT_MOI = 6.883
    private const val WHEEL_COF = 1.2


    val moduleTranslations: Array<Translation2d?>
    /** Returns an array of module translations.  */
    get() = arrayOf(
        Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY))
}

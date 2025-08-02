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

import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest.*
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team2471.off2025.Constants
import frc.team2471.off2025.OI
import frc.team2471.off2025.Robot
import frc.team2471.off2025.generated.TunerConstants
import frc.team2471.off2025.generated.TunerConstants.maxAngularSpeedRadPerSec
import frc.team2471.off2025.util.*
import frc.team2471.off2025.util.localization.QuixSwerveLocalizer
import frc.team2471.off2025.util.quix.Fiducials
import frc.team2471.off2025.util.vision.PhotonVisionCamera
import frc.team2471.off2025.util.vision.PipelineConfig
import frc.team2471.off2025.util.vision.QuixVisionCamera
import frc.team2471.off2025.util.vision.QuixVisionSim
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import kotlin.math.*

object Drive: SubsystemBase("Drive") {
    val io: DriveIO = DriveIOCTRE(TunerConstants.drivetrainConstants, *TunerConstants.moduleConfigs)
    val driveInputs = DriveIO.DriveIOInputs()

    @get:AutoLogOutput
    var pose: Pose2d
        get() = driveInputs.pose
        set(value) = io.resetPose(value)

    @get:AutoLogOutput
    var heading: Rotation2d
        get() = pose.rotation
        set(value) = io.resetHeading(value.measure)

    @get:AutoLogOutput
    val headingLatencyCompensated: Rotation2d
        get() = driveInputs.gyroInputs.yaw.asRotation2d

    val speeds: ChassisSpeeds
        get() = driveInputs.speeds

    val swerveKinematics = SwerveDriveKinematics(*TunerConstants.moduleTranslationsMeters.toTypedArray())


    val cameras: ArrayList<QuixVisionCamera> = arrayListOf(
        PhotonVisionCamera("FrontLeft", Constants.frontLeftCamPose, arrayOf(PipelineConfig())),
        PhotonVisionCamera("FrontRight", Constants.frontRightCamPose, arrayOf(PipelineConfig())),
        PhotonVisionCamera("BackLeft", Constants.backLeftCamPose, arrayOf(PipelineConfig())),
        PhotonVisionCamera("BackRight", Constants.backRightCamPose, arrayOf(PipelineConfig())),
    )
    val visionSim = QuixVisionSim(cameras, Fiducials.aprilTagFiducials)

    val localizer = QuixSwerveLocalizer(swerveKinematics, 0.0.degrees.asRotation2d /* this number can be anything */, driveInputs.modulePositions, Pose2d() /* this can be anything */, Fiducials.aprilTagFiducials, cameras)



    private val pathXController = PIDController(7.0, 0.0, 0.0)
    private val pathYController = PIDController(7.0, 0.0, 0.0)
    private val pathThetaController = PIDController(7.0, 0.0, 0.0).apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }

    private val autoDriveToPointController = PIDController(3.0, 0.0, 0.1)
    private val teleopDriveToPointController = PIDController(3.0, 0.0, 0.1)

    private val driveAtAngleRequest = FieldCentricFacingAngle().withDriveRequestType(SwerveModule.DriveRequestType.Velocity).apply {
        HeadingController = PhoenixPIDController(5.0, 0.0, 0.0).apply {
            enableContinuousInput(-Math.PI, Math.PI)
        }
        DriveRequestType = SwerveModule.DriveRequestType.Velocity
    }

    val gyroDisconnectedAlert = Alert("Gyro Disconnected", Alert.AlertType.kError)
    val moduleDisconnectedAlerts = Array<Triple<Alert, Alert, Alert>>(4) {
        Triple(
            Alert("Module $it Drive Motor Disconnected", Alert.AlertType.kError),
            Alert("Module $it Steer Motor Disconnected", Alert.AlertType.kError),
            Alert("Module $it Encoder Disconnected", Alert.AlertType.kError),
        )
    }

    init {
        println("inside Drive init")
        zeroGyro()

        localizer.trackAllTags()
    }

    override fun periodic() {
        io.updateInputs(driveInputs)
        Logger.processInputs("Drive", driveInputs)

        Logger.recordOutput("moduleTranslations", *TunerConstants.moduleTranslationsMeters.toTypedArray())
        gyroDisconnectedAlert.set(!driveInputs.gyroInputs.gyroConnected)
        driveInputs.moduleInputs.forEachIndexed { i, mInput ->
            // Alert if any swerve motor or encoder is disconnected
            val moduleAlert = moduleDisconnectedAlerts[i]
            moduleAlert.first.set(!mInput.driveConnected)
            moduleAlert.second.set(!mInput.steerConnected)
            moduleAlert.third.set(!mInput.encoderConnected)
        }

        //vision
        cameras.forEach {
            it.updateInputs()
        }
        localizer.updateWithLatestPoseEstimate()
        val odometryMeasurement = QuixSwerveLocalizer.SwerveOdometryMeasurement(headingLatencyCompensated, driveInputs.modulePositions)
        val visionMeasurements = cameras.map { it.latestMeasurement }.toCollection(ArrayList())
        localizer.update(odometryMeasurement, visionMeasurements, speeds)

        Logger.recordOutput("Swerve/Odometry", localizer.odometryPose)
        Logger.recordOutput("Swerve/Localizer Raw", localizer.rawPose)
        Logger.recordOutput("Swerve/Localizer", localizer.pose)
        Logger.recordOutput("Swerve/SingleTagPose", localizer.singleTagPose)




        if (Robot.isDisabled) {
            stop()
        }

        LoopLogger.record("Drive periodic()")
    }

    /**
     * Runs the drive at the desired velocity.
     * @param speeds Speeds in meters/sec
     */
    fun driveVelocity(speeds: ChassisSpeeds) {
        Logger.recordOutput("Drive/Wanted ChassisSpeeds", speeds.fieldToRobotCentric(heading))
        io.setDriveRequest(
            ApplyFieldSpeeds().apply{
                Speeds = speeds
                DriveRequestType = SwerveModule.DriveRequestType.Velocity
            }
        )
    }

    /**
     * Runs the drive at the desired voltage.
     * @param speedsInVolts Speeds in volts
     */
    fun driveVoltage(speedsInVolts: ChassisSpeeds) {
        io.setDriveRequest(
            ApplyFieldSpeeds().apply {
                Speeds = speedsInVolts
                DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage
            }
        )
    }

    fun stop() = driveVoltage(ChassisSpeeds())

    fun xPose() = io.setDriveRequest(SwerveDriveBrake())

    fun zeroGyro() {
        val wantedAngle = (if (isRedAlliance) 180.0.degrees else 0.0.degrees).asRotation2d
        println("zero gyro isRedAlliance  $isRedAlliance zeroing to ${wantedAngle.degrees} degrees")
        heading = wantedAngle
        println("heading: ${heading.degrees}")
    }

    fun updateSim() {
        if (isSim) io.updateSim()
    }

    fun brakeMode() = io.brakeMode()
    fun coastMode() = io.coastMode()

    fun poseAt(timestampSeconds: Double): Pose2d? = io.poseAt(timestampSeconds)

    fun setAngleOffsets() = runOnce {
        io.setAngleOffsets()
    }


    /**
     * Returns [ChassisSpeeds] with a percentage power from the driver controller.
     * Performs [OI.unsnapAndDesaturateJoystick] to undo axis snapping and does squaring/cubing on the vectors.
     */
    fun getChassisPercentSpeedsFromJoystick(): ChassisSpeeds {
        //make joystick pure circle
        val (cx, cy) = OI.unsnapAndDesaturateJoystick(OI.driveTranslationX, OI.driveTranslationY)

        //square drive input
        val power = hypot(cx, cy).square()
        val (x, y) = Pair(cx * power, cy * power)

        //cube rotation input
        val omega = OI.driveRotation.cube()

        return ChassisSpeeds(x, y, omega)
    }

    fun getChassisSpeedsFromJoystick(): ChassisSpeeds = getChassisPercentSpeedsFromJoystick().apply {
            vxMetersPerSecond *= TunerConstants.kSpeedAt12Volts.asMetersPerSecond
            vyMetersPerSecond *= TunerConstants.kSpeedAt12Volts.asMetersPerSecond
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

    fun driveAtAngle(angle: Rotation2d, translation: Translation2d) {
        io.setDriveRequest(
            driveAtAngleRequest.apply {
                VelocityX = translation.x
                VelocityY = translation.y
                TargetDirection = angle
            }
        )
    }


    fun driveToPoint(
        wantedPose: Pose2d,
        exitSupplier: (Distance, Angle) -> Boolean = { error, headingError -> error < 1.0.inches && headingError < 3.0.degrees },
        maxVelocity: LinearVelocity = TunerConstants.kSpeedAt12Volts
    ): Command {
        var distanceToPose: Double = Double.POSITIVE_INFINITY
        val pidController = if (Robot.isAutonomous) autoDriveToPointController else teleopDriveToPointController

        Logger.recordOutput("Drive/DriveToPoint Point", wantedPose)


        return run {
            val translationToPose = wantedPose.translation.minus(pose.translation)
            distanceToPose = translationToPose.norm
            val velocityOutput = min(abs(pidController.calculate(distanceToPose, 0.0)), maxVelocity.asMetersPerSecond)
            val wantedVelocity = translationToPose.normalize() * velocityOutput

            driveAtAngle(wantedPose.rotation, wantedVelocity)
        }.until {
            val distanceError = distanceToPose.meters
            val headingError = (wantedPose.rotation - pose.rotation).measure.absoluteValue()

            Logger.recordOutput("Drive/DriveToPoint DistanceError", distanceError)
            Logger.recordOutput("Drive/DriveToPoint HeadingError", headingError)

            exitSupplier(distanceError, headingError)
        }.finallyRun {
            stop()
            Logger.recordOutput("Drive/DriveToPoint Point", Pose2d())
        }.withName("DriveToPoint")
    }

    fun driveAlongChoreoPath(path: Trajectory<SwerveSample>, resetOdometry: Boolean = false, exitSupplier: (Double) -> Boolean = { it >= 1.0 }): Command {
        val totalTime = path.totalTime

        Logger.recordOutput("path name", path.name())
        Logger.recordOutput("path totalTime", totalTime)

        if (resetOdometry) {
            pose = path.getInitialPose(false).get()
        }

        var t = 0.0
        val timer = Timer()
        timer.start()

        return run {
            val currentPose = pose
            t = min(timer.get(), totalTime)
            val sample = path.sampleAt(t, false).get()
            val wantedPose = sample.pose
            val wantedSpeeds = sample.chassisSpeeds
            val moduleForcesX = sample.moduleForcesX()
            val moduleForcesY = sample.moduleForcesY()

            Logger.recordOutput("Path Time", t)
            Logger.recordOutput("Path Pose", wantedPose)
            Logger.recordOutput("Path Speeds", wantedSpeeds)
            Logger.recordOutput("Path Module Forces X", moduleForcesX)
            Logger.recordOutput("Path Module Forces Y", moduleForcesY)

            wantedSpeeds.apply {
                vxMetersPerSecond += pathXController.calculate(currentPose.x, wantedPose.x)
                vyMetersPerSecond += pathYController.calculate(currentPose.y, wantedPose.y)
                omegaRadiansPerSecond += pathThetaController.calculate(currentPose.rotation.radians, sample.heading)
            }
            io.setDriveRequest(
                ApplyFieldSpeeds().apply {
                    Speeds = wantedSpeeds
                    WheelForceFeedforwardsX = moduleForcesX
                    WheelForceFeedforwardsY = moduleForcesY
                    DriveRequestType = SwerveModule.DriveRequestType.Velocity
                }
            )
        }.until {
            exitSupplier(t / totalTime)
        }.finallyRun {
            // Tell drivetrain to apply no output
            stop()
            // Publish empty data to show that the path is done
            Logger.recordOutput("path name", "")
            Logger.recordOutput("Path Pose", Pose2d(null, null))
        }.withName("DriveAlongChoreoPath")
    }






    // sysID

    private val translationSysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            7.0.volts,
            5.0.seconds
        ) { state: SysIdRoutineLog.State -> SignalLogger.writeString("SysIdTranslation_State", state.toString())},
        Mechanism({ output: Voltage -> io.setDriveRequest(SysIdSwerveTranslation().withVolts(output))}, null, this)
    )
    //used to find driveAtAngleRequest PID
    private val rotationSysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            Units.Volts.of(Math.PI / 6).per(Units.Second),
            Math.PI.volts,
            5.0.seconds
        ) { state: SysIdRoutineLog.State -> SignalLogger.writeString("SysIdRotation_State", state.toString())},
        Mechanism({ output: Voltage ->
            /* output is actually radians per second, but SysId only supports "volts" */
            io.setDriveRequest(SysIdSwerveRotation().withRotationalRate(output.asVolts))
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.asVolts)
        }, null, this)
    )
    private val steerSysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            7.0.volts,
            null
        ) { state: SysIdRoutineLog.State -> SignalLogger.writeString("SysIdSteer_State", state.toString()) },
        Mechanism({ volts: Voltage? -> io.setDriveRequest(SysIdSwerveSteerGains().withVolts(volts)) }, null, this)
    )

    fun sysIDTranslationDynamic(direction: SysIdRoutine.Direction) = translationSysIdRoutine.dynamic(direction).finallyWait(1.0)
    fun sysIDTranslationQuasistatic(direction: SysIdRoutine.Direction) = translationSysIdRoutine.quasistatic(direction).finallyWait(1.0)
    fun sysIDRotationDynamic(direction: SysIdRoutine.Direction) = rotationSysIdRoutine.dynamic(direction).finallyWait(1.0)
    fun sysIDRotationQuasistatic(direction: SysIdRoutine.Direction) = rotationSysIdRoutine.quasistatic(direction).finallyWait(1.0)
    fun sysIDSteerDynamic(direction: SysIdRoutine.Direction) = steerSysIdRoutine.dynamic(direction).finallyWait(1.0)
    fun sysIDSteerQuasistatic(direction: SysIdRoutine.Direction) = steerSysIdRoutine.quasistatic(direction).finallyWait(1.0)

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


    override fun simulationPeriodic() {
        LoopLogger.record("b4 Drive Sim piodic")
        GlobalScope.launch {
            visionSim.updatePose(pose)
        }
        LoopLogger.record("Drive Sim piodic")
    }
}

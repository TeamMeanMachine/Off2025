package frc.team2471.off2025.subsystems.drive

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Preferences
import frc.team2471.off2025.util.*
import frc.team2471.off2025.util.logged.LoggedTalonFX
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlin.jvm.optionals.getOrNull

class DriveIOCTRE(
    driveConstants: SwerveDrivetrainConstants,
    vararg moduleConstants: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
): SwerveDrivetrain<LoggedTalonFX, LoggedTalonFX, CANcoder>(
    { deviceId: Int, canbus: String? -> LoggedTalonFX(deviceId, canbus) },
    { deviceId: Int, canbus: String? -> LoggedTalonFX(deviceId, canbus) },
    { deviceId: Int, canbus: String? -> CANcoder(deviceId, canbus) },
    driveConstants,
    *moduleConstants
), DriveIO {
    val moduleSignals = Array(4) {
        val m = getModule(it)
        val drive = m.driveMotor
        val steer = m.steerMotor
        val encoder = m.encoder
        ModuleSignals(
            drive.velocity,
            drive.acceleration,
            drive.supplyCurrent,
            drive.statorCurrent,
            drive.motorVoltage,
            drive.deviceTemp,
            steer.velocity,
            steer.acceleration,
            steer.supplyCurrent,
            steer.statorCurrent,
            steer.motorVoltage,
            steer.deviceTemp,
            encoder.absolutePosition
        )
    }
    val gyroSignals = GyroSignals(
        pigeon2.yaw,
        pigeon2.angularVelocityZWorld,
        pigeon2.pitch,
        pigeon2.angularVelocityXWorld,
        pigeon2.roll,
        pigeon2.angularVelocityYWorld,
        pigeon2.accelerationX,
        pigeon2.accelerationY,
        pigeon2.gravityVectorX,
        pigeon2.gravityVectorY
    )

    init {

    }

    override fun updateInputs(inputs: DriveIO.DriveIOInputs) {
        LoopLogger.record("a drive updateInputs")
        val s = this.stateCopy
        inputs.pose = s.Pose
        inputs.speeds = s.Speeds.robotToFieldCentric(s.Pose.rotation)
        inputs.moduleStates = s.ModuleStates
        inputs.moduleTargets = s.ModuleTargets
        inputs.modulePositions = s.ModulePositions
        inputs.rawHeading = s.RawHeading
        inputs.timestamp = s.Timestamp
        inputs.odometryPeriod = s.OdometryPeriod
        inputs.successfulDaqs = s.SuccessfulDaqs
        inputs.failedDaqs = s.FailedDaqs

        LoopLogger.record("a stateCopy")
        inputs.moduleInputs = Array(4) {
            val m = moduleSignals[it]
            DriveIO.ModuleInput(
                driveConnected = m.driveConnectedDebouncer.calculate(BaseStatusSignal.refreshAll(*m.allDriveSignals).isOK),
                driveVelocity = m.driveVelocity.valueAsDouble.rotationsPerSecond,
                driveAccel = m.driveAcceleration.valueAsDouble.rotationsPerSecondPerSecond,
                driveSupplyCurrentAmps = m.driveSupplyCurrent.valueAsDouble,
                driveStatorCurrentAmps = m.driveStatorCurrent.valueAsDouble,
                driveAppliedVolts = m.driveAppliedVolts.valueAsDouble,
                driveTemp = m.driveTemperature.valueAsDouble,

                steerConnected = m.steerConnectedDebouncer.calculate(BaseStatusSignal.refreshAll(*m.allSteerSignals).isOK),
                steerVelocity = m.steerVelocity.valueAsDouble.rotationsPerSecond,
                steerAccel = m.steerAcceleration.valueAsDouble.rotationsPerSecondPerSecond,
                steerSupplyCurrentAmps = m.steerSupplyCurrent.valueAsDouble,
                steerStatorCurrentAmps = m.steerStatorCurrent.valueAsDouble,
                steerAppliedVolts = m.steerAppliedVolts.valueAsDouble,
                steerTemp = m.steerTemperature.valueAsDouble,

                encoderConnected = m.encoderConnectedDebouncer.calculate(BaseStatusSignal.refreshAll(*m.allEncoderSignals).isOK),
                encoderAbsoluteAngle = m.encoderAbsolutePosition.valueAsDouble.rotations
            )
        }
        LoopLogger.record("a module")

        val g = gyroSignals
        inputs.gyroInputs.apply {
            gyroConnected = g.gyroConnectedDebouncer.calculate(BaseStatusSignal.refreshAll(*g.allSignals).isOK)

            yaw = BaseStatusSignal.getLatencyCompensatedValueAsDouble(g.yaw, g.yawRate).degrees
            pitch = BaseStatusSignal.getLatencyCompensatedValueAsDouble(g.pitch, g.pitchRate).degrees
            roll = BaseStatusSignal.getLatencyCompensatedValueAsDouble(g.roll, g.rollRate).degrees

            yawRate = g.yawRate.valueAsDouble.degreesPerSecond
            pitchRate = g.pitchRate.valueAsDouble.degreesPerSecond
            rollRate = g.rollRate.valueAsDouble.degreesPerSecond

            xAccel = g.xAccel.valueAsDouble.Gs - g.xGrav.valueAsDouble.Gs
            yAccel = g.yAccel.valueAsDouble.Gs - g.yGrav.valueAsDouble.Gs
        }
        LoopLogger.record("UpdateInputs Drive")
    }

    override fun setDriveRequest(request: SwerveRequest) = this.setControl(request)

    override fun resetPose(pose: Pose2d?) {
        val safePose = pose ?: Pose2d()
        resetTranslation(safePose.translation)
        resetHeading(safePose.rotation.measure)
    }

    override fun updateSim() {
        if (isSim) {
            updateSimState(0.02, 12.0)
        } else {
            DriverStation.reportError("DriveIOCTRE.updateSim() called while robot is real", true)
            throw Error("DriveIOCTRE.updateSim() called while robot is real")
        }
    }

    override fun brakeMode() {
        modules.forEach {
            it.steerMotor.brakeMode()
            it.driveMotor.brakeMode()
        }
    }

    @OptIn(DelicateCoroutinesApi::class)
    override fun coastMode() {
        modules.forEach {
            it.steerMotor.coastMode()
            it.steerMotor.coastMode()
        }
    }

    override fun setAngleOffsets() {
        val offsets = modules.map { it.encoder.setCANCoderAngle(0.0.degrees) }
        offsets.forEachIndexed { i, offset ->
            Preferences.setDouble("Module $i Offset", offset.asDegrees)
        }
    }

    override fun poseAt(timestampSeconds: Double): Pose2d? = samplePoseAt(timestampSeconds).getOrNull()


    override fun resetHeading(angle: Angle) {
        println("reseting heading to $angle")
//        resetRotation(180.0.degrees.asRotation2d)
//        resetRotation(angle.asRotation2d - 180.0.degrees.asRotation2d)
        pigeon2.setYaw(angle)
    }


    class ModuleSignals(
        val driveVelocity: BaseStatusSignal,
        val driveAcceleration: BaseStatusSignal,
        val driveSupplyCurrent: BaseStatusSignal,
        val driveStatorCurrent: BaseStatusSignal,
        val driveAppliedVolts: BaseStatusSignal,
        val driveTemperature: BaseStatusSignal,
        val steerVelocity: BaseStatusSignal,
        val steerAcceleration: BaseStatusSignal,
        val steerSupplyCurrent: BaseStatusSignal,
        val steerStatorCurrent: BaseStatusSignal,
        val steerAppliedVolts: BaseStatusSignal,
        val steerTemperature: BaseStatusSignal,
        val encoderAbsolutePosition: BaseStatusSignal
    ) {
        val allDriveSignals: Array<BaseStatusSignal> = arrayOf(
            driveVelocity,
            driveAcceleration,
            driveSupplyCurrent,
            driveStatorCurrent,
            driveAppliedVolts,
            driveTemperature
        )
        val allSteerSignals: Array<BaseStatusSignal> = arrayOf(
            steerVelocity,
            steerAcceleration,
            steerSupplyCurrent,
            steerStatorCurrent,
            steerAppliedVolts,
            steerTemperature
        )
        val allEncoderSignals: Array<BaseStatusSignal> = arrayOf(
            encoderAbsolutePosition
        )

        val driveConnectedDebouncer = Debouncer(0.5)
        val steerConnectedDebouncer = Debouncer(0.5)
        val encoderConnectedDebouncer = Debouncer(0.5)
    }

    class GyroSignals(
        val yaw: BaseStatusSignal,
        val yawRate: BaseStatusSignal,
        val pitch: BaseStatusSignal,
        val pitchRate: BaseStatusSignal,
        val roll: BaseStatusSignal,
        val rollRate: BaseStatusSignal,
        val xAccel: BaseStatusSignal,
        val yAccel: BaseStatusSignal,
        val xGrav: BaseStatusSignal,
        val yGrav: BaseStatusSignal
    ) {
        val allSignals: Array<BaseStatusSignal> =
            arrayOf(yaw, yawRate, pitch, pitchRate, roll, rollRate, xAccel, yAccel, xGrav, yGrav)
        val gyroConnectedDebouncer = Debouncer(0.5)
    }
}
package frc.team2471.off2025.subsystems.drive

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Preferences
import frc.team2471.off2025.util.*
import kotlin.math.roundToInt

class DriveIOCTRE(
    driveConstants: SwerveDrivetrainConstants,
    vararg moduleConstants: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
): SwerveDrivetrain<TalonFX, TalonFX, CANcoder>(
    { deviceId: Int, canbus: String? -> TalonFX(deviceId, canbus) },
    { deviceId: Int, canbus: String? -> TalonFX(deviceId, canbus) },
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

        val alerts = arrayListOf<Alert>()
        modules.forEachIndexed { i, module ->
            if (module.encoder.isConnected) {
                val encoderConfigs = CANcoderConfiguration().apply {
                    PhoenixUtil.tryUntilOk(5) { module.encoder.configurator.apply(this) }
                }
                val prefsOffset = Preferences.getDouble("Module $i Offset", Double.NaN).degrees
                val encoderOffset = encoderConfigs.MagnetSensor.magnetOffsetMeasure
                if (prefsOffset.asDegrees.isNaN()) {
                    //couldn't find prefs
                    println("module $i has missing Preferences, setting to encoders Offset: ${encoderOffset.asDegrees.roundToInt()}")
                    Preferences.setDouble("Module $i Offset", encoderOffset.asDegrees)
                    alerts.add(Alert("Module $i had missing Networktables prefs", Alert.AlertType.kWarning))
                } else if (prefsOffset.asDegrees.roundToInt() != encoderOffset.asDegrees.roundToInt()) {
                    //offsets are different, default to prefs
                    println("module $i has conflicting offsets. prefsOffset: ${prefsOffset.asDegrees.round(2)} encoders Offset: ${encoderOffset.asDegrees.round(2)}")
                    module.encoder.setMagnetSensorOffset(prefsOffset)
                    alerts.add(Alert("Module $i had conflicting offsets", Alert.AlertType.kWarning))
                }
            } else {
                alerts.add(Alert("Module $i CANCoder Disconnected On INIT", Alert.AlertType.kError))
            }
        }

        alerts.forEach { it.set(true) }
    }

    override fun updateInputs(inputs: DriveIO.DriveIOInputs) {
        val s = this.stateCopy
        inputs.pose = s.Pose
        inputs.speeds = s.Speeds
        inputs.moduleStates = s.ModuleStates
        inputs.moduleTargets = s.ModuleTargets
        inputs.modulePositions = s.ModulePositions
        inputs.rawHeading = s.RawHeading
        inputs.timestamp = s.Timestamp
        inputs.odometryPeriod = s.OdometryPeriod
        inputs.successfulDaqs = s.SuccessfulDaqs
        inputs.failedDaqs = s.FailedDaqs

        inputs.moduleInputs = Array(4) {
            val m = moduleSignals[it]
            DriveIO.ModuleInput.Empty.apply {
                driveConnected = m.driveConnectedDebouncer.calculate(BaseStatusSignal.refreshAll(*m.allDriveSignals).isOK)
                driveVelocity = m.driveVelocity.valueAsDouble.rotationsPerSecond
                driveAccel = m.driveAcceleration.valueAsDouble.rotationsPerSecondPerSecond
                driveSupplyCurrentAmps = m.driveSupplyCurrent.valueAsDouble
                driveStatorCurrentAmps = m.driveStatorCurrent.valueAsDouble
                driveAppliedVolts = m.driveAppliedVolts.valueAsDouble
                driveTemp = m.driveTemperature.valueAsDouble

                steerConnected = m.steerConnectedDebouncer.calculate(BaseStatusSignal.refreshAll(*m.allSteerSignals).isOK)
                steerVelocity = m.steerVelocity.valueAsDouble.rotationsPerSecond
                steerAccel = m.steerAcceleration.valueAsDouble.rotationsPerSecondPerSecond
                steerSupplyCurrentAmps = m.steerSupplyCurrent.valueAsDouble
                steerStatorCurrentAmps = m.steerStatorCurrent.valueAsDouble
                steerAppliedVolts = m.steerAppliedVolts.valueAsDouble
                steerTemp = m.steerTemperature.valueAsDouble

                encoderConnected = m.encoderConnectedDebouncer.calculate(BaseStatusSignal.refreshAll(*m.allEncoderSignals).isOK)
                encoderAbsoluteAngle = m.encoderAbsolutePosition.valueAsDouble.rotations
            }
        }

        val g = gyroSignals
        inputs.gyroInputs.apply {
            gyroConnected = g.gyroConnectedDebouncer.calculate(BaseStatusSignal.refreshAll(*g.allSignals).isOK)
            yaw = g.yaw.valueAsDouble.degrees
            yawRate = g.yawRate.valueAsDouble.degreesPerSecond
            pitch = g.pitch.valueAsDouble.degrees
            pitchRate = g.pitchRate.valueAsDouble.degreesPerSecond
            roll = g.roll.valueAsDouble.degrees
            rollRate = g.rollRate.valueAsDouble.degreesPerSecond
            xAccel = g.xAccel.valueAsDouble.Gs - g.xGrav.valueAsDouble.Gs
            yAccel = g.yAccel.valueAsDouble.Gs - g.yGrav.valueAsDouble.Gs
        }
    }

    override fun setDriveRequest(request: SwerveRequest) = this.setControl(request)

    override fun resetPose(pose: Pose2d?) {
        val safePose = pose ?: Pose2d()
        resetPosition(safePose.translation)
        resetHeading(safePose.rotation.measure)
    }

    override fun resetPosition(translation: Translation2d?) {
        //if translation is null, reset to (0, 0)
        resetTranslation(translation ?: Translation2d())
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
        configNeutralMode(NeutralModeValue.Brake)
    }

    override fun coastMode() {
        configNeutralMode(NeutralModeValue.Coast)
    }

    override fun setAngleOffsets() {
        val offsets = modules.map { it.encoder.setCANCoderAngle(0.0.degrees) }
        offsets.forEachIndexed { i, offset ->
            Preferences.setDouble("Module $i Offset", offset.asDegrees)
        }
    }


    override fun resetHeading(angle: Angle) {
        println("reseting heading to $angle")
        resetRotation(angle.asRotation2d)
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
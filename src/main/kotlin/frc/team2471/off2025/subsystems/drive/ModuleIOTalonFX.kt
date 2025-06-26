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

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.*
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.team2471.off2025.generated.TunerConstants
import frc.team2471.off2025.subsystems.drive.ModuleIO.ModuleIOInputs
import frc.team2471.off2025.util.*
import java.util.*

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 *
 * Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
class ModuleIOTalonFX(private val constants: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>) : ModuleIO {
    // Hardware objects
    private val driveMotor = TalonFX(constants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName)
    private val steerMotor = TalonFX(constants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName)
    private val cancoder = CANcoder(constants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName)

    private var cancoderOffset: Angle = Double.NaN.degrees
        get() {
            if (field.asDegrees.isNaN()) {
                val cfg = CANcoderConfiguration()
                PhoenixUtil.tryUntilOk(5) { cancoder.configurator.refresh(cfg) }
                return cfg.MagnetSensor.MagnetOffset.rotations
            }
            return field
        }
        set(value) {
            val initialConfigs = CANcoderConfiguration()
            cancoder.configurator.refresh(initialConfigs)

            initialConfigs.MagnetSensor.MagnetOffset = value.asRotations

            cancoder.configurator.apply(initialConfigs, 0.0)
            field = value
        }

    //DutyCycle (percentage) control requests
    private val dutyCycleRequest = DutyCycleOut(0.0)

    // Voltage control requests
    private val voltageRequest = VoltageOut(0.0)
    private val positionVoltageRequest = PositionVoltage(0.0)
    private val velocityVoltageRequest = VelocityVoltage(0.0)

    // Torque-current control requests
    private val torqueCurrentRequest = TorqueCurrentFOC(0.0)
    private val positionTorqueCurrentRequest = PositionTorqueCurrentFOC(0.0)
    private val velocityTorqueCurrentRequest = VelocityTorqueCurrentFOC(0.0)

    // Timestamp inputs from Phoenix thread
    private val timestampQueue: Queue<Double> = OdometrySignalThread.makeTimestampQueue()

    // Inputs from drive motor
    private val drivePosition: StatusSignal<Angle> = driveMotor.position
    private val drivePositionQueue: Queue<Double> = OdometrySignalThread.registerSignal(driveMotor.position)
    private val driveVelocity: StatusSignal<AngularVelocity> = driveMotor.velocity
    private val driveAppliedVoltage: StatusSignal<Voltage> = driveMotor.motorVoltage
    private val driveCurrent: StatusSignal<Current> = driveMotor.statorCurrent

    // Inputs from turn motor
    private val turnAbsolutePositionS: StatusSignal<Angle> = cancoder.absolutePosition
    private val turnPositionS: StatusSignal<Angle> = steerMotor.position
    private val turnPositionQueue: Queue<Double> = OdometrySignalThread.registerSignal(steerMotor.position)
    private val turnVelocity: StatusSignal<AngularVelocity> = steerMotor.velocity
    private val turnAppliedVoltsS: StatusSignal<Voltage> = steerMotor.motorVoltage
    private val turnCurrent: StatusSignal<Current> = steerMotor.statorCurrent

    // Connection debouncers
    private val driveConnectedDebounce = Debouncer(0.5)
    private val turnConnectedDebounce = Debouncer(0.5)
    private val turnEncoderConnectedDebounce = Debouncer(0.5)

    init {
        // Configure drive motor
        val driveConfig = constants.DriveMotorInitialConfigs.apply {
            Slot0 = constants.DriveMotorGains
            Feedback.apply {
                SensorToMechanismRatio = constants.DriveMotorGearRatio
            }
            TorqueCurrent.apply{
                PeakForwardTorqueCurrent = constants.SlipCurrent
                PeakReverseTorqueCurrent = -constants.SlipCurrent
            }
            CurrentLimits.apply {
                StatorCurrentLimit = constants.SlipCurrent
                StatorCurrentLimitEnable = true
            }
            MotorOutput.apply {
                Inverted = if (constants.DriveMotorInverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
            }
        }
        PhoenixUtil.tryUntilOk(5) { driveMotor.configurator.apply(driveConfig, 0.25) }
        PhoenixUtil.tryUntilOk(5) { driveMotor.setPosition(0.0, 0.25) }

        // Configure turn motor
        val turnConfig = constants.SteerMotorInitialConfigs.apply {
            Slot0 = constants.SteerMotorGains
            Feedback.apply {
                FeedbackRemoteSensorID = constants.EncoderId
                FeedbackSensorSource = when (constants.FeedbackSource) {
                    SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder
                    SwerveModuleConstants.SteerFeedbackType.FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder
                    SwerveModuleConstants.SteerFeedbackType.SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder
                    else -> throw RuntimeException("You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.")
                }
                RotorToSensorRatio = constants.SteerMotorGearRatio
            }
            MotionMagic.apply {
                MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio
                MotionMagicAcceleration = MotionMagicCruiseVelocity / 0.100
                MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio
                MotionMagicExpo_kA = 0.1
            }
            ClosedLoopGeneral.apply {
                ContinuousWrap = true
            }
            MotorOutput.apply {
                Inverted = if (constants.SteerMotorInverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
            }
        }
        PhoenixUtil.tryUntilOk(5) { steerMotor.configurator.apply(turnConfig, 0.25) }

        // Configure CANCoder
        val cancoderConfig = constants.EncoderInitialConfigs.apply {
            MagnetSensor.apply {
                MagnetOffset = cancoderOffset.asRotations
                SensorDirection = if (constants.EncoderInverted) SensorDirectionValue.Clockwise_Positive else SensorDirectionValue.CounterClockwise_Positive
            }
        }
        PhoenixUtil.tryUntilOk(5) { cancoder.configurator.apply(cancoderConfig, 0.25) }

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll( //signals needed for odometry
            OdometrySignalThread.ODOMETRY_FREQUENCY,
            drivePosition,
            turnPositionS
        )
        BaseStatusSignal.setUpdateFrequencyForAll( //signals not needed for odometry
            50.0,
            driveVelocity,
            driveAppliedVoltage,
            driveCurrent,
            turnAbsolutePositionS,
            turnVelocity,
            turnAppliedVoltsS,
            turnCurrent
        )
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, steerMotor) //Only publish data if an update frequency has been set for it
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        // Refresh all signals (obtains new data from CAN bus)
        val driveIsConnected = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVoltage, driveCurrent).isOK
        val turnIsConnected = BaseStatusSignal.refreshAll(turnPositionS, turnVelocity, turnAppliedVoltsS, turnCurrent).isOK
        val turnEncoderIsConnected = BaseStatusSignal.refreshAll(turnAbsolutePositionS).isOK

        inputs.apply {
            // Update drive inputs
            driveConnected = driveConnectedDebounce.calculate(driveIsConnected)
            drivePositionRad = drivePosition.value.asRadians
            driveVelocityRadPerSec = driveVelocity.value.asRadiansPerSecond
            driveAppliedVolts = driveAppliedVoltage.valueAsDouble
            driveCurrentAmps = driveCurrent.valueAsDouble

            // Update turn inputs
            turnConnected = turnConnectedDebounce.calculate(turnIsConnected)
            turnPosition = turnPositionS.value.asRotation2d
            turnVelocityRadPerSec = turnVelocity.value.asRadiansPerSecond
            turnAppliedVolts = turnAppliedVoltsS.valueAsDouble
            turnCurrentAmps = turnCurrent.valueAsDouble

            turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderIsConnected)
            turnAbsolutePosition = turnAbsolutePositionS.value.asRotation2d

            // Update odometry inputs
            odometryTimestamps = timestampQueue.stream().mapToDouble { it }.toArray()
            odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble { Units.rotationsToRadians(it) }.toArray()
            odometryTurnPositions = turnPositionQueue.stream().toArray().map { Rotation2d.fromRotations(it as Double) }.toTypedArray()
        }

        timestampQueue.clear()
        drivePositionQueue.clear()
        turnPositionQueue.clear()
    }

    private fun setDrivePercentage(output: Double) =
        driveMotor.setControl(dutyCycleRequest.withOutput(output))


    private fun setDriveVoltage(output: Double) =
        driveMotor.setControl(
            when (constants.DriveMotorClosedLoopOutput!!) {
                SwerveModuleConstants.ClosedLoopOutputType.Voltage -> voltageRequest.withOutput(output)
                SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output)
            }
        )

    private fun setTurnVoltage(output: Double) =
        steerMotor.setControl(
            when (constants.SteerMotorClosedLoopOutput!!) {
                SwerveModuleConstants.ClosedLoopOutputType.Voltage -> voltageRequest.withOutput(output)
                SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output)
            }
        )

    private fun setDriveVelocity(velocityRadPerSec: Double) {
        val velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec)
        driveMotor.setControl(
            when (constants.DriveMotorClosedLoopOutput!!) {
                SwerveModuleConstants.ClosedLoopOutputType.Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec)
                SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec)
            }
        )
    }

    private fun setTurnPosition(rotation: Rotation2d) =
        steerMotor.setControl(
            when (constants.SteerMotorClosedLoopOutput!!) {
                SwerveModuleConstants.ClosedLoopOutputType.Voltage -> positionVoltageRequest.withPosition(rotation.rotations)
                SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(rotation.rotations)
            }
        )

    override fun setVelocity(state: SwerveModuleState) {
        setTurnPosition(state.angle)
        setDriveVelocity(state.speedMetersPerSecond)
    }

    override fun setVoltage(state: SwerveModuleState) {
        setTurnPosition(state.angle)
        setDriveVoltage(state.speedMetersPerSecond)
    }

    override fun setPercentage(state: SwerveModuleState) {
        setTurnPosition(state.angle)
        setDrivePercentage(state.speedMetersPerSecond)
    }

    override fun setCANCoderAngle(angle: Angle): Angle {
        val initialConfigs = CANcoderConfiguration()
        PhoenixUtil.tryUntilOk(5) { cancoder.configurator.refresh(initialConfigs) }

        val initialPosition = cancoder.absolutePosition.valueAsDouble.rotations
        val initialOffset = initialConfigs.MagnetSensor.MagnetOffset.rotations
        println("initial position $initialPosition initial offset $initialOffset")

        val rawPosition = initialPosition - initialOffset
        val newOffset = (angle - rawPosition)
        println("rawPosition $rawPosition new offset $newOffset")

        setCANCoderOffset(newOffset)
        return cancoderOffset
    }

    override fun setCANCoderOffset(offset: Angle) {
        cancoderOffset = offset
    }

    override fun getCANCoderOffset(): Angle = cancoderOffset

    override fun brakeMode() {
        driveMotor.setNeutralMode(NeutralModeValue.Brake, 0.0)
        steerMotor.setNeutralMode(NeutralModeValue.Brake, 0.0)
    }

    override fun coastMode() {
        driveMotor.setNeutralMode(NeutralModeValue.Coast, 0.0)
        steerMotor.setNeutralMode(NeutralModeValue.Coast, 0.0)
    }

}

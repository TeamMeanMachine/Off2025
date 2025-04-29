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
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.team2471.off2025.generated.TunerConstants
import frc.team2471.off2025.subsystems.drive.ModuleIO.ModuleIOInputs
import frc.team2471.off2025.util.PhoenixUtil
import frc.team2471.off2025.util.asRotations
import frc.team2471.off2025.util.rotations
import java.util.*

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 *
 * Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
class ModuleIOTalonFX(private val constants: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>) :
    ModuleIO {
    // Hardware objects
    private val driveTalon: TalonFX = TalonFX(constants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName)
    private val turnTalon: TalonFX = TalonFX(constants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName)
    private val cancoder: CANcoder = CANcoder(constants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName)

    // Voltage control requests
    private val voltageRequest = VoltageOut(0.0)
    private val positionVoltageRequest = PositionVoltage(0.0)
    private val velocityVoltageRequest = VelocityVoltage(0.0)

    // Torque-current control requests
    private val torqueCurrentRequest = TorqueCurrentFOC(0.0)
    private val positionTorqueCurrentRequest = PositionTorqueCurrentFOC(0.0)
    private val velocityTorqueCurrentRequest = VelocityTorqueCurrentFOC(0.0)

    // Timestamp inputs from Phoenix thread
    private val timestampQueue: Queue<Double?>

    // Inputs from drive motor
    private val drivePosition: StatusSignal<Angle?>
    private val drivePositionQueue: Queue<Double?>
    private val driveVelocity: StatusSignal<AngularVelocity?>
    private val driveAppliedVoltage: StatusSignal<Voltage?>
    private val driveCurrent: StatusSignal<Current?>

    // Inputs from turn motor
    private val turnAbsolutePositionS: StatusSignal<Angle?>
    private val turnPositionS: StatusSignal<Angle?>
    private val turnPositionQueue: Queue<Double?>
    private val turnVelocity: StatusSignal<AngularVelocity?>
    private val turnAppliedVoltsS: StatusSignal<Voltage?>
    private val turnCurrent: StatusSignal<Current?>

    // Connection debouncers
    private val driveConnectedDebounce = Debouncer(0.5)
    private val turnConnectedDebounce = Debouncer(0.5)
    private val turnEncoderConnectedDebounce = Debouncer(0.5)

    init {
        // Configure drive motor
        val driveConfig = constants.DriveMotorInitialConfigs.apply {
            MotorOutput.NeutralMode = NeutralModeValue.Brake
            Slot0 = constants.DriveMotorGains
            Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio
            TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent
            TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent
            CurrentLimits.StatorCurrentLimit = constants.SlipCurrent
            CurrentLimits.StatorCurrentLimitEnable = true
            MotorOutput.Inverted = if (constants.DriveMotorInverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
        }
        PhoenixUtil.tryUntilOk(5) { driveTalon.configurator.apply(driveConfig, 0.25) }
        PhoenixUtil.tryUntilOk(5) { driveTalon.setPosition(0.0, 0.25) }

        // Configure turn motor
        val turnConfig = TalonFXConfiguration().apply {
            MotorOutput.NeutralMode = NeutralModeValue.Brake
            Slot0 = constants.SteerMotorGains
            Feedback.FeedbackRemoteSensorID = constants.EncoderId
            Feedback.FeedbackSensorSource = when (constants.FeedbackSource) {
                SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder
                SwerveModuleConstants.SteerFeedbackType.FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder
                SwerveModuleConstants.SteerFeedbackType.SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder
                else -> throw RuntimeException(
                    "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers."
                )
            }
            Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio
            MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio
            MotionMagic.MotionMagicAcceleration = MotionMagic.MotionMagicCruiseVelocity / 0.100
            MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio
            MotionMagic.MotionMagicExpo_kA = 0.1
            ClosedLoopGeneral.ContinuousWrap = true
            MotorOutput.Inverted = if (constants.SteerMotorInverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
        }
        PhoenixUtil.tryUntilOk(5) { turnTalon.configurator.apply(turnConfig, 0.25) }

        // Configure CANCoder
        val cancoderConfig = constants.EncoderInitialConfigs.apply {
            MagnetSensor.MagnetOffset = constants.EncoderOffset
            MagnetSensor.SensorDirection = if (constants.EncoderInverted) SensorDirectionValue.Clockwise_Positive else SensorDirectionValue.CounterClockwise_Positive
        }
        cancoder.configurator.apply(cancoderConfig)

        // Create timestamp queue
        timestampQueue = PhoenixOdometryThread.instance.makeTimestampQueue()

        // Create drive status signals
        drivePosition = driveTalon.position
        drivePositionQueue = PhoenixOdometryThread.instance.registerSignal(driveTalon.position)
        driveVelocity = driveTalon.velocity
        driveAppliedVoltage = driveTalon.motorVoltage
        driveCurrent = driveTalon.statorCurrent

        // Create turn status signals
        turnAbsolutePositionS = cancoder.absolutePosition
        turnPositionS = turnTalon.position
        turnPositionQueue = PhoenixOdometryThread.instance.registerSignal(turnTalon.position)
        turnVelocity = turnTalon.velocity
        turnAppliedVoltsS = turnTalon.motorVoltage
        turnCurrent = turnTalon.statorCurrent

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
            Drive.ODOMETRY_FREQUENCY, drivePosition, turnPositionS
        )
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVoltage,
            driveCurrent,
            turnAbsolutePositionS,
            turnVelocity,
            turnAppliedVoltsS,
            turnCurrent
        )
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon)
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        // Refresh all signals
        val driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVoltage, driveCurrent)
        val turnStatus = BaseStatusSignal.refreshAll(turnPositionS, turnVelocity, turnAppliedVoltsS, turnCurrent)
        val turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePositionS)

        inputs.apply {
            // Update drive inputs
            driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK)
            drivePositionRad = Units.rotationsToRadians(drivePosition.valueAsDouble)
            driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.valueAsDouble)
            driveAppliedVolts = driveAppliedVoltage.valueAsDouble
            driveCurrentAmps = driveCurrent.valueAsDouble

            // Update turn inputs
            turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK)
            turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK)
            turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePositionS.valueAsDouble)
            turnPosition = Rotation2d.fromRotations(turnPositionS.valueAsDouble)
            turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.valueAsDouble)
            turnAppliedVolts = turnAppliedVoltsS.valueAsDouble
            turnCurrentAmps = turnCurrent.valueAsDouble

            // Update odometry inputs
            odometryTimestamps = timestampQueue.stream().mapToDouble { value: Double? -> value!! }.toArray()
            odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble { value: Double? -> Units.rotationsToRadians(value!!) }.toArray()
            odometryTurnPositions = turnPositionQueue.stream().toArray().map { Rotation2d.fromDegrees(it as Double) }.toTypedArray()
        }

        timestampQueue.clear()
        drivePositionQueue.clear()
        turnPositionQueue.clear()
    }

    override fun setDriveOpenLoop(output: Double) {
        driveTalon.setControl(
            when (constants.DriveMotorClosedLoopOutput) {
                SwerveModuleConstants.ClosedLoopOutputType.Voltage -> voltageRequest.withOutput(output)
                SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output)
            }
        )
    }

    override fun setTurnOpenLoop(output: Double) {
        turnTalon.setControl(
            when (constants.SteerMotorClosedLoopOutput) {
                SwerveModuleConstants.ClosedLoopOutputType.Voltage -> voltageRequest.withOutput(output)
                SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output)
            }
        )
    }

    override fun setDriveVelocity(velocityRadPerSec: Double) {
        val velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec)
        driveTalon.setControl(
            when (constants.DriveMotorClosedLoopOutput) {
                SwerveModuleConstants.ClosedLoopOutputType.Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec)
                SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec)
            }
        )
    }

    override fun setTurnPosition(rotation: Rotation2d) {
        turnTalon.setControl(
            when (constants.SteerMotorClosedLoopOutput) {
                SwerveModuleConstants.ClosedLoopOutputType.Voltage -> positionVoltageRequest.withPosition(rotation.rotations)
                SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(rotation.rotations)
            }
        )
    }


    override fun setCANCoderAngle(angle: Angle) {
        val initialConfigs = CANcoderConfiguration()
        cancoder.configurator.refresh(initialConfigs)
        val initialPosition = cancoder.absolutePosition.valueAsDouble.rotations
        val initialOffset = initialConfigs.MagnetSensor.MagnetOffset.rotations
        println("inital position $initialPosition initial offset $initialOffset")
        val rawPosition = initialPosition + initialOffset
        val newOffset = (rawPosition - angle)
        println("rawPosition $rawPosition new offset $newOffset")
        initialConfigs.MagnetSensor.MagnetOffset = newOffset.asRotations
        cancoder.configurator.apply(initialConfigs)
    }

}

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

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.team2471.off2025.generated.TunerConstants
import frc.team2471.off2025.subsystems.drive.ModuleIO.ModuleIOInputs
import kotlin.math.abs
import kotlin.math.sign

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
class ModuleIOSim(constants: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>) : ModuleIO {
    // Create drive and turn sim models
    private val driveSim: DCMotorSim = DCMotorSim(LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio), DRIVE_GEARBOX)
    private val turnSim: DCMotorSim = DCMotorSim(LinearSystemId.createDCMotorSystem(TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio), TURN_GEARBOX)

    private var driveClosedLoop = false
    private var turnClosedLoop = false
    private val driveController = PIDController(DRIVE_KP, 0.0, DRIVE_KD)
    private val turnController = PIDController(TURN_KP, 0.0, TURN_KD)
    private var driveFFVolts = 0.0
    private var driveAppliedVolts = 0.0
    private var turnAppliedVolts = 0.0

    init {
        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI)
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.angularVelocityRadPerSec)
        } else {
            driveController.reset()
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.angularPositionRad)
        } else {
            turnController.reset()
        }

        // Update simulation state
        driveSim.inputVoltage = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0)
        turnSim.inputVoltage = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0)
        driveSim.update(0.02)
        turnSim.update(0.02)

        // Update drive inputs
        inputs.driveConnected = true
        inputs.drivePositionRad = driveSim.angularPositionRad
        inputs.driveVelocityRadPerSec = driveSim.angularVelocityRadPerSec
        inputs.driveAppliedVolts = driveAppliedVolts
        inputs.driveCurrentAmps = abs(driveSim.currentDrawAmps)

        // Update turn inputs
        inputs.turnConnected = true
        inputs.turnEncoderConnected = true
        inputs.turnAbsolutePosition = Rotation2d(turnSim.angularPositionRad)
        inputs.turnPosition = Rotation2d(turnSim.angularPositionRad)
        inputs.turnVelocityRadPerSec = turnSim.angularVelocityRadPerSec
        inputs.turnAppliedVolts = turnAppliedVolts
        inputs.turnCurrentAmps = abs(turnSim.currentDrawAmps)

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
        inputs.odometryTimestamps = doubleArrayOf(Timer.getFPGATimestamp())
        inputs.odometryDrivePositionsRad = doubleArrayOf(inputs.drivePositionRad)
        inputs.odometryTurnPositions = arrayOf(inputs.turnPosition)
    }

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

    private fun setDrivePercentage(output: Double) {
        setDriveVoltage(output * 12.0)
    }

    private fun setDriveVoltage(output: Double) {
        driveClosedLoop = false
        driveAppliedVolts = output
    }

    private fun setTurnOpenLoop(output: Double) {
        turnClosedLoop = false
        turnAppliedVolts = output
    }

    private fun setDriveVelocity(velocityRadPerSec: Double) {
        driveClosedLoop = true
        driveFFVolts = DRIVE_KS * sign(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec
        driveController.setSetpoint(velocityRadPerSec)
    }

    private fun setTurnPosition(rotation: Rotation2d) {
        turnClosedLoop = true
        turnController.setSetpoint(rotation.radians)
    }

    companion object {
        // TunerConstants doesn't support separate sim constants, so they are declared locally
        private const val DRIVE_KP = 0.05
        private const val DRIVE_KD = 0.0
        private const val DRIVE_KS = 0.0
        private const val DRIVE_KV_ROT = 0.91035 // Same units as TunerConstants: (volt * secs) / rotation
        private val DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT)
        private const val TURN_KP = 8.0
        private const val TURN_KD = 0.0
        private val DRIVE_GEARBOX: DCMotor = TunerConstants.driveMotor
        private val TURN_GEARBOX: DCMotor = TunerConstants.steerMotor
    }
}

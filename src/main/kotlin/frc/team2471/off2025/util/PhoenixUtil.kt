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
package frc.team2471.off2025.util

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DriverStation
import java.util.function.Supplier

object PhoenixUtil {
    /** Attempts to run the command until no error is produced.  */
    fun tryUntilOk(maxAttempts: Int, command: Supplier<StatusCode>) {
        for (i in 0..<maxAttempts) {
            val error = command.get()
            if (error.isOK || isSim) break
            if (i == maxAttempts - 1) DriverStation.reportError("tryUntilOk() reached max attempts of $maxAttempts and failed with error: ${error.description}", true)
        }
    }

    fun getMagnetSensorOffsetFromCANcoderID(id: Int, canBus: String = ""): Angle {
        return CANcoder(id, canBus).getMagnetSensorOffset()
    }
}

/**
 * Swerve request to set the individual module states.
 *
 * If no value is passed in, the module is set to its current angle with 0 speed
 */
class ApplyModuleStates(vararg val moduleStates: SwerveModuleState? = arrayOf()): SwerveRequest {
    override fun apply(
        parameters: SwerveDrivetrain.SwerveControlParameters?,
        vararg modulesToApply: SwerveModule<*, *, *>
    ): StatusCode {
        modulesToApply.forEachIndexed { index, module ->
            val wantedState = moduleStates.getOrNull(index) ?: SwerveModuleState(0.0, module.currentState.angle)
            module.apply(SwerveModule.ModuleRequest().withState(wantedState))
        }

        return StatusCode.OK
    }
}

/** Grabs the MagnetOffset from the [CANcoder]. */
fun CANcoder.getMagnetSensorOffset(): Angle {
    if (this.isConnected || isSim) return 0.0.degrees
    val initialConfigs = CANcoderConfiguration()
    PhoenixUtil.tryUntilOk(5) { this.configurator.refresh(initialConfigs) }

    return initialConfigs.MagnetSensor.MagnetOffset.rotations
}

/**
 * Applies the MagnetOffset config to the [CANcoder] while not changing other configuration values.
 * This is probably a backing call
 */
fun CANcoder.setMagnetSensorOffset(offset: Angle) {
    if (isReal) {
        val initialConfigs = CANcoderConfiguration()
        PhoenixUtil.tryUntilOk(5) { this.configurator.refresh(initialConfigs) }

        initialConfigs.MagnetSensor.MagnetOffset = offset.asRotations

        this.configurator.apply(initialConfigs, 0.0)
    }
}

/**
 * Sets the [CANcoder]s MagnetOffset config so its current position equals the specified angle.
 * This is probably a backing call
 */
fun CANcoder.setCANCoderAngle(angle: Angle): Angle {
    val initialPosition = this.absolutePosition.valueAsDouble.rotations
    val initialOffset = getMagnetSensorOffset()
    println("initial position $initialPosition initial offset $initialOffset")

    val rawPosition = initialPosition - initialOffset
    val newOffset = (angle - rawPosition)
    println("rawPosition $rawPosition new offset $newOffset")

    this.setMagnetSensorOffset(newOffset)
    return newOffset
}

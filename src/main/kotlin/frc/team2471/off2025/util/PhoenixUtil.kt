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
import edu.wpi.first.units.measure.Angle
import java.util.function.Supplier

object PhoenixUtil {
    /** Attempts to run the command until no error is produced.  */
    fun tryUntilOk(maxAttempts: Int, command: Supplier<StatusCode>) {
        for (i in 0..<maxAttempts) {
            val error = command.get()
            if (error.isOK) break
        }
    }
}
/** Sets the [CANcoder]s MagnetOffset config so its current position equals the specified angle. */
fun CANcoder.setCANCoderAngle(angle: Angle): Angle {
    val initialConfigs = CANcoderConfiguration()
    PhoenixUtil.tryUntilOk(5) { this.configurator.refresh(initialConfigs) }

    val initialPosition = this.absolutePosition.valueAsDouble.rotations
    val initialOffset = initialConfigs.MagnetSensor.MagnetOffset.rotations
    println("initial position $initialPosition initial offset $initialOffset")

    val rawPosition = initialPosition - initialOffset
    val newOffset = (angle - rawPosition)
    println("rawPosition $rawPosition new offset $newOffset")

    this.setMagnetSensorOffset(newOffset)
    return newOffset
}

/** Applies the MagnetOffset config to the [CANcoder] while not changing other configuration values. */
fun CANcoder.setMagnetSensorOffset(offset: Angle) {
    val initialConfigs = CANcoderConfiguration()
    this.configurator.refresh(initialConfigs)

    initialConfigs.MagnetSensor.MagnetOffset = offset.asRotations

    this.configurator.apply(initialConfigs, 0.0)
}

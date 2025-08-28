package frc.team2471.off2025.util.ctre

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Angle
import frc.team2471.off2025.util.isReal
import frc.team2471.off2025.util.isSim
import frc.team2471.off2025.util.units.asRotations
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.rotations

/** Grabs the MagnetOffset from the [CANcoder]. */
fun CANcoder.getMagnetSensorOffset(): Angle {
    if (!this.isConnected || isSim) return 0.0.degrees
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
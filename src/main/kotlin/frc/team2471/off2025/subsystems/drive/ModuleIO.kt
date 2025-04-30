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

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.Angle
import org.littletonrobotics.junction.AutoLog

interface ModuleIO {
    @AutoLog
    open class ModuleIOInputs {
        @JvmField var driveConnected: Boolean = false
        @JvmField var drivePositionRad: Double = 0.0
        @JvmField var driveVelocityRadPerSec: Double = 0.0
        @JvmField var driveAppliedVolts: Double = 0.0
        @JvmField var driveCurrentAmps: Double = 0.0

        @JvmField var turnConnected: Boolean = false
        @JvmField var turnEncoderConnected: Boolean = false
        @JvmField var turnAbsolutePosition: Rotation2d = Rotation2d()
        @JvmField var turnPosition: Rotation2d = Rotation2d()
        @JvmField var turnVelocityRadPerSec: Double = 0.0
        @JvmField var turnAppliedVolts: Double = 0.0
        @JvmField var turnCurrentAmps: Double = 0.0

        @JvmField var odometryTimestamps: DoubleArray = doubleArrayOf()
        @JvmField var odometryDrivePositionsRad: DoubleArray = doubleArrayOf()
        @JvmField var odometryTurnPositions: Array<Rotation2d> = arrayOf<Rotation2d>()
    }

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ModuleIOInputs) {}

    /** Run the drive motor at the specified open loop value.  */
    fun setDriveOpenLoop(output: Double) {}

    /** Run the turn motor at the specified open loop value.  */
    fun setTurnOpenLoop(output: Double) {}

    /** Run the drive motor at the specified velocity.  */
    fun setDriveVelocity(velocityRadPerSec: Double) {}

    /** Run the turn motor to the specified rotation.  */
    fun setTurnPosition(rotation: Rotation2d) {}

    /** Set encoder offset angle to a specified angle.  */
    fun setCANCoderAngle(angle: Angle) {}
}

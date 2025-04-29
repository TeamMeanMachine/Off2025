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
package frc.team2471.off2025.subsystems.drive.gyro

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog

interface GyroIO {
    @AutoLog
    open class GyroIOInputs {
        @JvmField var connected: Boolean = false
        @JvmField var yawPosition: Rotation2d = Rotation2d()
        @JvmField var yawVelocityRadPerSec: Double = 0.0
        @JvmField var odometryYawTimestamps: DoubleArray = doubleArrayOf()
        @JvmField var odometryYawPositions: Array<Rotation2d> = arrayOf<Rotation2d>()
    }

    fun updateInputs(inputs: GyroIOInputs) {}
}

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

import com.studica.frc.AHRS
import com.studica.frc.AHRS.NavXComType
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.team2471.off2025.subsystems.drive.Drive
import frc.team2471.off2025.subsystems.drive.PhoenixOdometryThread
import java.util.*

/** IO implementation for NavX.  */
class GyroIONavX : GyroIO {
    private val navX = AHRS(NavXComType.kMXP_SPI, Drive.ODOMETRY_FREQUENCY.toInt().toByte().toInt())
    private val yawTimestampQueue: Queue<Double> = PhoenixOdometryThread.makeTimestampQueue()
    private val yawPositionQueue: Queue<Double> = PhoenixOdometryThread.registerSignal { -navX.yaw.toDouble() }

    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.connected = navX.isConnected
        inputs.yawPosition = Rotation2d.fromDegrees(-navX.yaw.toDouble())
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.rawGyroZ.toDouble())

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble { it }.toArray()
        inputs.odometryYawPositions = yawPositionQueue.stream().toArray().map { Rotation2d.fromDegrees(it as Double) }.toTypedArray()

        yawTimestampQueue.clear()
        yawPositionQueue.clear()
    }
}

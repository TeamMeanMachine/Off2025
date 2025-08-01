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
package frc.team2471.off2025.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.team2471.off2025.subsystems.drive.Drive
import frc.team2471.off2025.util.LoopLogger
import frc.team2471.off2025.util.isBlueAlliance
import frc.team2471.off2025.util.isRedAlliance

object DriveCommands {
    private const val ANGLE_KP = 5.0
    private const val ANGLE_KD = 0.4
    private const val ANGLE_MAX_VELOCITY = 8.0
    private const val ANGLE_MAX_ACCELERATION = 20.0
    const val FF_START_DELAY = 0.5 // Secs
    const val FF_RAMP_RATE = 0.1 // Volts/Sec
    const val WHEEL_RADIUS_MAX_VELOCITY = 2.0 // Rad/Sec
    const val WHEEL_RADIUS_RAMP_RATE = 0.3 // Rad/Sec^2

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    @JvmStatic
    fun joystickDrive(): Command {
        return Commands.run({
            LoopLogger.record("b4 joystickDrive")
            // Get linear velocity
            val chassisSpeeds = Drive.getChassisSpeedsFromJoystick().apply {
                if (isRedAlliance) {
                    vxMetersPerSecond *= -1.0
                    vyMetersPerSecond *= -1.0
                }
            }

            //send it
            Drive.driveVelocity(chassisSpeeds)

            LoopLogger.record("joystickDrive")
            },
            Drive
        )
    }


    class WheelRadiusCharacterizationState {
        var positions: DoubleArray = DoubleArray(4)
        var lastAngle: Rotation2d = Rotation2d()
        var gyroDelta: Double = 0.0
    }
}

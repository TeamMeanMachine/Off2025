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

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.team2471.off2025.subsystems.drive.Drive
import java.util.function.Supplier

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
            // Get linear velocity
            val chassisSpeeds = Drive.getChassisSpeedsFromJoystick()//.fieldToRobotCentric(if (isRedAlliance) -Drive.rotation.plus(Rotation2d(Math.PI)) else -Drive.rotation)

            // Convert to field relative speeds & send command
            Drive.driveVelocity(chassisSpeeds)

            },
            Drive
        )
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    fun joystickDriveAtAngle(goalAngle: Supplier<Rotation2d>): Command {
        // Create PID controller

        val angleController = ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION)
            )
        angleController.enableContinuousInput(-Math.PI, Math.PI)

        // Construct command
        return Commands.run(
             {
                 // Get linear velocity from joysticks
                 val chassisSpeeds = Drive.getChassisSpeedsFromJoystick()

                 // Calculate angular speed
                 chassisSpeeds.omegaRadiansPerSecond = angleController.calculate(Drive.rotation.radians, goalAngle.get().radians)

                 Drive.driveVelocity(chassisSpeeds)
            },
            Drive
        ) // Reset PID controller when command starts
            .beforeStarting({ angleController.reset(Drive.rotation.radians) })
    }



    fun zeroSwerveOffsets(): Command = Commands.sequence(
        Commands.runOnce({
            Drive.driveVelocity(ChassisSpeeds(0.0, 0.0, 0.0))

        })
    )

    class WheelRadiusCharacterizationState {
        var positions: DoubleArray = DoubleArray(4)
        var lastAngle: Rotation2d = Rotation2d()
        var gyroDelta: Double = 0.0
    }
}

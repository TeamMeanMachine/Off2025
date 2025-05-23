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
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.team2471.off2025.OI
import frc.team2471.off2025.subsystems.drive.Drive
import frc.team2471.off2025.util.asDegrees
import frc.team2471.off2025.util.radians
import java.text.DecimalFormat
import java.text.NumberFormat
import java.util.*
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

object DriveCommands {
    private const val DEADBAND = 0.1
    private const val ANGLE_KP = 5.0
    private const val ANGLE_KD = 0.4
    private const val ANGLE_MAX_VELOCITY = 8.0
    private const val ANGLE_MAX_ACCELERATION = 20.0
    private const val FF_START_DELAY = 0.5 // Secs
    private const val FF_RAMP_RATE = 0.1 // Volts/Sec
    private const val WHEEL_RADIUS_MAX_VELOCITY = 2.0 // Rad/Sec
    private const val WHEEL_RADIUS_RAMP_RATE = 0.3 // Rad/Sec^2

    private fun getLinearVelocityFromJoysticks(x: Double, y: Double): Translation2d {
        val linearMagnitude = hypot(x, y)
        val linearDirection = Rotation2d(atan2(y, x))

        // Return new linear velocity
        return Pose2d(Translation2d(), linearDirection)
            .transformBy(Transform2d(linearMagnitude, 0.0, Rotation2d()))
            .translation
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    @JvmStatic
    fun joystickDrive(): Command {
        return Commands.run({
            // Get linear velocity
            val linearVelocity = getLinearVelocityFromJoysticks(OI.driveTranslationX, OI.driveTranslationY)

            val omega = OI.driveRotation

            // Convert to field relative speeds & send command
            val speeds =
                ChassisSpeeds(
                    linearVelocity.x * Drive.maxLinearSpeedMetersPerSec,
                    linearVelocity.y * Drive.maxLinearSpeedMetersPerSec,
                    omega * Drive.maxAngularSpeedRadPerSec
                )
            val isFlipped = DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == Alliance.Red

            Drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, if (isFlipped) Drive.rotation.plus(Rotation2d(Math.PI)) else Drive.rotation))
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
                // Get linear velocity
                val linearVelocity = getLinearVelocityFromJoysticks(OI.driveTranslationX, OI.driveTranslationY)

                // Calculate angular speed
                val omega = angleController.calculate(Drive.rotation.radians, goalAngle.get().radians)

                // Convert to field relative speeds & send command
                val speeds = ChassisSpeeds(
                        linearVelocity.x * Drive.maxLinearSpeedMetersPerSec,
                        linearVelocity.y * Drive.maxLinearSpeedMetersPerSec,
                        omega
                    )

                val isFlipped = DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == Alliance.Red

                Drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, if (isFlipped) Drive.rotation.plus(Rotation2d(Math.PI)) else Drive.rotation))
            },
            Drive
        ) // Reset PID controller when command starts
            .beforeStarting({ angleController.reset(Drive.rotation.radians) })
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     *
     * This command should only be used in voltage control mode.
     */
    @JvmStatic
    fun feedforwardCharacterization(): Command {
        val velocitySamples: MutableList<Double> = LinkedList<Double>()
        val voltageSamples: MutableList<Double> = LinkedList<Double>()
        val timer = Timer()

        return Commands.sequence( // Reset data
            Commands.runOnce(
                {
                    velocitySamples.clear()
                    voltageSamples.clear()
                }),  // Allow modules to orient

            Commands.run({ Drive.runCharacterization(0.0) }, Drive).withTimeout(FF_START_DELAY),  // Start timer

            Commands.runOnce({ timer.restart() }),  // Accelerate and gather data

            Commands.run(
                {
                    val voltage = timer.get() * FF_RAMP_RATE
                    Drive.runCharacterization(voltage)
                    velocitySamples.add(Drive.fFCharacterizationVelocity)
                    voltageSamples.add(voltage)
                },
                Drive
            ) // When cancelled, calculate and print results

                .finallyDo(
                    Runnable {
                        val n = velocitySamples.size
                        var sumX = 0.0
                        var sumY = 0.0
                        var sumXY = 0.0
                        var sumX2 = 0.0
                        for (i in 0..<n) {
                            velocitySamples[i] = velocitySamples[i] + velocitySamples[i]
                            voltageSamples[i] = voltageSamples[i] + voltageSamples[i]
                            sumXY += velocitySamples[i] * voltageSamples[i]
                            sumX2 += velocitySamples[i] * velocitySamples[i]
                        }
                        val kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX)
                        val kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX)

                        val formatter: NumberFormat = DecimalFormat("#0.00000")
                        println("********** Drive FF Characterization Results **********")
                        println("\tkS: " + formatter.format(kS))
                        println("\tkV: " + formatter.format(kV))
                    })
        )
    }

    /** Measures the robot's wheel radius by spinning in a circle.  */
    @JvmStatic
    fun wheelRadiusCharacterization(): Command {
        val limiter = SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE)
        val state = WheelRadiusCharacterizationState()

        return Commands.parallel( // Drive control sequence
            Commands.sequence( // Reset acceleration limiter
                Commands.runOnce({
                    limiter.reset(0.0)
                }),  // Turn in place, accelerating up to full speed

                Commands.run({
                    val speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY)
                    Drive.runVelocity(ChassisSpeeds(0.0, 0.0, speed))
                             },
                    Drive
                )
            ),  // Measurement sequence

            Commands.sequence( // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),  // Record starting measurement

                Commands.runOnce(
                     {
                        state.positions = Drive.wheelRadiusCharacterizationPositions
                        state.lastAngle = Drive.rotation
                        state.gyroDelta = 0.0
                    }),  // Update gyro delta

                Commands.run(
                     {
                        val rotation = Drive.rotation
                        state.gyroDelta += abs(rotation.minus(state.lastAngle).radians)
                        state.lastAngle = rotation
                    }) // When cancelled, calculate and print results

                    .finallyDo(
                        Runnable {
                            val positions = Drive.wheelRadiusCharacterizationPositions
                            var wheelDelta = 0.0
                            for (i in 0..3) {
                                wheelDelta += abs(positions[i] - state.positions[i]) / 4.0
                            }
                            val wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta

                            val formatter: NumberFormat = DecimalFormat("#0.000")
                            println("********** Wheel Radius Characterization Results **********")
                            println("\tWheel Delta: " + wheelDelta.radians.asDegrees + " degrees")
                            println("\tGyro Delta: " + state.gyroDelta.radians.asDegrees + " degrees")
                            println(("\tWheel Radius: " + formatter.format(wheelRadius) + " meters, " + formatter.format(Units.metersToInches(wheelRadius)) + " inches"))
                        })
            )
        )
    }

    fun zeroSwerveOffsets(): Command = Commands.sequence(
        Commands.runOnce({
            Drive.runVelocity(ChassisSpeeds(0.0, 0.0, 0.0))

        })
    )

    private class WheelRadiusCharacterizationState {
        var positions: DoubleArray = DoubleArray(4)
        var lastAngle: Rotation2d = Rotation2d()
        var gyroDelta: Double = 0.0
    }
}

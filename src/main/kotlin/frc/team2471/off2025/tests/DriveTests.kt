package frc.team2471.off2025.tests

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.team2471.off2025.OI
import frc.team2471.off2025.OI.dPad
import frc.team2471.off2025.Drive
import frc.team2471.off2025.util.beforeRun
import frc.team2471.off2025.util.runCommand
import frc.team2471.off2025.util.translation
import org.littletonrobotics.junction.Logger


fun joystickTest(): Command {
    val joystickValues = mutableListOf<Translation2d>()


    return Commands.run({
        val translation = OI.rawDriveTranslation
        if (translation.norm > 0.5) {
            if (Math.random() > 0.5) {
//                joystickValues.clear()
                joystickValues.add(translation)
            }
            Logger.recordOutput("joystickValues", *joystickValues.toTypedArray())
            Logger.recordOutput("rawJoystickValue", Translation2d(OI.driveTranslationX, OI.driveTranslationY))
        }
    }, Drive)
}

fun velocityVoltTest(): Command {
    var v = 0.0
    var upPressed = false
    var downPressed = false
    return runCommand(Drive) {
        if (OI.driverController.dPad == OI.Direction.UP) {
            upPressed = true
        } else if (OI.driverController.dPad == OI.Direction.DOWN) {
            downPressed = true
        }
        if (OI.driverController.dPad != OI.Direction.UP && upPressed) {
            upPressed = false
            v += 0.005
        }
        if (OI.driverController.dPad != OI.Direction.DOWN && downPressed) {
            downPressed = false
            v -= 0.005
        }
        println("v: $v velocity: ${Drive.speeds.translation.norm}")
        Drive.driveVoltage(ChassisSpeeds(v, 0.0, 0.0))
    }
}

/** Measures the robot's wheel radius by spinning in a circle.
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
                Drive.driveVelocity(ChassisSpeeds(0.0, 0.0, speed))
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
                        val wheelRadius = (state.gyroDelta * TunerConstants.DRIVE_BASE_RADIUS) / wheelDelta

                        val formatter: NumberFormat = DecimalFormat("#0.000")
                        println("********** Wheel Radius Characterization Results **********")
                        println("\tWheel Delta: " + wheelDelta.radians.asDegrees + " degrees")
                        println("\tGyro Delta: " + state.gyroDelta.radians.asDegrees + " degrees")
                        println(("\tWheel Radius: " + formatter.format(wheelRadius) + " meters, " + formatter.format(
                            Units.metersToInches(wheelRadius)) + " inches"))
                    })
        )
    )
}*/

/*
*
 * Measures the velocity feedforward constants for the drive motors.
 *
 *
 * This command should only be used in voltage control mode.
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
}*/

fun slipCurrentTest(): Command {
    val timer = edu.wpi.first.wpilibj.Timer()
    return runCommand(Drive) {
        val volts = timer.get() * 0.01
        Logger.recordOutput("Test/StaterCurrent", Drive.modules.first().driveMotor.statorCurrent.valueAsDouble)
        Logger.recordOutput("Test/Volts", volts)
        Logger.recordOutput("Test/Velocity", Drive.modules.first().driveMotor.velocity.valueAsDouble)
        Drive.driveVoltage(ChassisSpeeds(volts, 0.0, 0.0))
    }.beforeRun() {
        timer.restart()
    }
}
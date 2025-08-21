package frc.team2471.off2025

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team2471.off2025.util.*
import java.lang.IllegalStateException
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.math.sqrt
import kotlin.math.withSign

object OI: Subsystem {
    val driverController = CommandXboxController(0)
    val operatorController = CommandXboxController(1)

    val deadbandDriver = 0.08
    val deadbandOperator = 0.1

    val driveTranslationX: Double
        get() = -driverController.leftY.deadband(deadbandDriver)

    val driveTranslationY: Double
        get() = -driverController.leftX.deadband(deadbandDriver)

    val driveRotation: Double
        get() = -driverController.rightX

    val driveLeftTrigger: Double
        get() = driverController.leftTriggerAxis

    val driveLeftTriggerFullPress: Boolean
        get() = driverController.leftTriggerAxis > 0.95

    val driveRightTrigger: Double
        get() = driverController.rightTriggerAxis

    val driveRightTriggerFullPress: Boolean
        get() = driverController.rightTriggerAxis > 0.95

    val operatorLeftTrigger: Double
        get() = operatorController.leftTriggerAxis

    val operatorLeftY: Double
        get() = operatorController.leftY.deadband(deadbandOperator)

    val operatorLeftX: Double
        get() = operatorController.leftX.deadband(deadbandOperator)

    val operatorRightTrigger: Double
        get() = operatorController.rightTriggerAxis

    val operatorRightX: Double
        get() = operatorController.rightX.deadband(deadbandOperator)

    val operatorRightY: Double
        get() = operatorController.rightY.deadband(deadbandOperator)

    private val driverDebouncer = Debouncer(0.05)
    private val operatorDebouncer = Debouncer(0.05)


    init {
        // TODO: map the right trigger to setting the open loop voltage of the shooter motor.  full trigger should be MAX_VELOCITY
        driverController.rightTrigger(0.1).whileTrue(
            runCommand
            { Shooter.shooterVelocitySetpoint = (driverController.rightTriggerAxis * Shooter.MAX_VELOCITY) }.finallyRun
            { Shooter.stop() })

        // TODO: map a to 0 RPM velocity, b to 1250 RPM, and y to 3000 RPM
        driverController.a().whileTrue(runCommand { Shooter.shooterVelocitySetpoint = 0.0})
        driverController.b().whileTrue(runCommand { Shooter.shooterVelocitySetpoint = 1250.0})
        driverController.y().whileTrue(runCommand { Shooter.shooterVelocitySetpoint = 3000.0})
    }

    /**
     * Removes the 90 degree "snap" that Xbox joysticks do at extreme magnitudes and prevents magnitudes over 1
     */
    fun unsnapAndDesaturateJoystick(rawX: Double, rawY: Double): Pair<Double, Double> {
        return if (hypot(rawX, rawY) > 1.0) {
            //magnitude is > 1, something is being "snapped" or is inaccurate
            if (rawX.absoluteValue >= 1.0) {
                //x not trustworthy
                Pair(sqrt(1 - rawY.square()).withSign(rawX), rawY)
            } else if (rawY.absoluteValue >= 1.0) {
                //y not trustworthy
                Pair(rawX, sqrt(1 - rawX.square()).withSign(rawY))
            } else {
                //both kinda trustworthy so perform weighted average for smooth motion. larger values are more untrustworthy
                val xCalc = sqrt(1 - rawY.square()).withSign(rawX)
                val yCalc = sqrt(1 - rawX.square()).withSign(rawY)
                val yConfidence = rawX.absoluteValue
                val xConfidence = rawY.absoluteValue
                val totalWeight = (yConfidence + xConfidence)
                val x = (rawX * xConfidence + xCalc * yConfidence) / totalWeight
                val y = (rawY * yConfidence + yCalc * xConfidence) / totalWeight

//                println(hypot(x, y))
                Pair(x, y)
            }
        } else {
            Pair(rawX, rawY)
        }
    }

    inline val CommandXboxController.a: Boolean get() = this.hid.aButton
    inline val CommandXboxController.b: Boolean get() = this.hid.bButton
    inline val CommandXboxController.x: Boolean get() = this.hid.xButton
    inline val CommandXboxController.y: Boolean get() = this.hid.yButton

    inline val CommandXboxController.start: Boolean get() = this.hid.startButton
    inline val CommandXboxController.back: Boolean get() = this.hid.backButton

    inline val CommandXboxController.dPad: Direction get() = when (this.hid.pov) {
            -1 -> Direction.IDLE
            0 -> Direction.UP
            45 -> Direction.UP_RIGHT
            90 -> Direction.RIGHT
            135 -> Direction.DOWN_RIGHT
            180 -> Direction.DOWN
            225 -> Direction.DOWN_LEFT
            270 -> Direction.LEFT
            315 -> Direction.UP_LEFT
            else -> throw IllegalStateException("Invalid DPAD value ${this.hid.pov}")
    }

    enum class Direction { IDLE, UP, UP_RIGHT, RIGHT, DOWN_RIGHT, DOWN, DOWN_LEFT, LEFT, UP_LEFT }
}
package frc.team2471.off2025

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team2471.off2025.commands.DriveCommands
import frc.team2471.off2025.subsystems.Armavator
import frc.team2471.off2025.subsystems.drive.Drive
import frc.team2471.off2025.util.*
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.math.sqrt
import kotlin.math.withSign

object OI: Subsystem {
    val driverController = MeanCommandXboxController(0, false)
    val operatorController = MeanCommandXboxController(1)

    val deadbandDriver = 0.08
    val deadbandOperator = 0.1

    val driveTranslationX: Double
        get() = driverController.leftY.deadband(deadbandDriver)

    val driveTranslationY: Double
        get() = driverController.leftX.deadband(deadbandDriver)

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

    private val driverNotConnectedAlert: Alert = Alert("DRIVER JOYSTICK DISCONNECTED", Alert.AlertType.kError)
    private val operatorNotConnectedAlert: Alert = Alert("OPERATOR JOYSTICK DISCONNECTED", Alert.AlertType.kError)
    private val driverDebouncer = Debouncer(0.05)
    private val operatorDebouncer = Debouncer(0.05)



    init {
        // Default command, normal field-relative drive
        Drive.defaultCommand = DriveCommands.joystickDrive()



        // Lock to 0° when A button is held
        driverController.a().whileTrue(Drive.driveAtAngle { Rotation2d() })

        driverController.b().whileTrue(Drive.driveToPoint(Pose2d(2.0, 2.0, 90.0.degrees.asRotation2d)))

        // Switch to X pattern when X button is pressed
        driverController.x().onTrue(Commands.runOnce({ Drive.xPose() }, Drive))

        driverController.povUp().onTrue(Commands.runOnce({ Armavator.setElevatorPercentOut(0.1) }))
        driverController.povUp().onFalse(Commands.runOnce({ Armavator.setElevatorPercentOut(0.0) }))
        driverController.povDown().onTrue(Commands.runOnce({ Armavator.setElevatorPercentOut(-0.1) }))
        driverController.povDown().onFalse(Commands.runOnce({ Armavator.setElevatorPercentOut(0.0) }))

        // Reset gyro to 0° when B button is pressed
        driverController.back().onTrue(
            runOnceCommand(Drive) {
                Drive.zeroGyro()
            }.ignoringDisable(true))

        // Reset position to zero
        driverController.start().onTrue(
            runOnceCommand(Drive) {
                Drive.pose = Pose2d(Translation2d(), Drive.rotation)
            }.ignoringDisable(true))
    }

    override fun periodic() {
        driverNotConnectedAlert.set(driverDebouncer.calculate(driverController.isConnected))
        operatorNotConnectedAlert.set(operatorDebouncer.calculate(operatorController.isConnected))
    }

    /**
     * Uses distance formula to remove the 90 degree "snap" that Xbox joysticks do at extreme magnitudes and prevents magnitudes over 1.
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

    inline val CommandXboxController.rightBumper: Boolean get() = this.hid.rightBumperButton
    inline val CommandXboxController.leftBumper: Boolean get() = this.hid.leftBumperButton

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
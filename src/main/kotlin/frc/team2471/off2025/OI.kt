package frc.team2471.off2025

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team2471.off2025.FieldManager.onOpposingAllianceSide
import frc.team2471.off2025.FieldManager.reflectAcrossField
import frc.team2471.off2025.util.*
import frc.team2471.off2025.util.units.asRotation2d
import frc.team2471.off2025.util.units.degrees
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.math.sqrt
import kotlin.math.withSign

object OI: SubsystemBase("OI") {
    val driverController = MeanCommandXboxController(0, false)
    val operatorController = MeanCommandXboxController(1)

    val deadbandDriver = 0.08
    val deadbandOperator = 0.1

    val driveTranslationX: Double
        get() = driverController.leftY.deadband(deadbandDriver)

    val driveTranslationY: Double
        get() = driverController.leftX.deadband(deadbandDriver)

    val driveRotation: Double
        get() = -driverController.rightX.deadband(deadbandDriver)

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
        println("inside OI init")
        // Default command, normal field-relative drive
        Drive.defaultCommand = Drive.joystickDrive()


        driverController.a().onTrue(runOnceCommand(Armavator){ Armavator.goToPose(Pose.DRIVE)})
        driverController.b().onTrue(runOnceCommand(Armavator){ Armavator.goToPose(Pose.SCORE_L3)})
    //    driverController.rightBumper().onTrue(runOnceCommand{Intake.intakeState = IntakeState.INTAKING})
    //    driverController.leftBumper().onTrue(runOnceCommand{Intake.intakeState = IntakeState.HOLDING})
    //    driverController.rightTrigger().onTrue(runOnceCommand{Intake.intakeState = IntakeState.SCORING})
    //    driverController.leftTrigger().onTrue(runOnceCommand{Intake.intakeState = IntakeState.REVERSING})



        driverController.y().whileTrue(defer {
            Drive.joystickDriveAlongLine(
                FieldManager.bargeAlignPoints.first.reflectAcrossField { Drive.localizer.pose.onOpposingAllianceSide() },
                FieldManager.bargeAlignPoints.second.reflectAcrossField { Drive.localizer.pose.onOpposingAllianceSide() },
                (if (Drive.heading.degrees.absoluteValue > 90.0) 180.0 else 0.0).degrees.asRotation2d
            ) })

/*        driverController.a().onTrue(runOnce {
            Drive.questSimConnected = !Drive.questSimConnected
            println("questSimConnected = ${Drive.questSimConnected}")
        })*/

        driverController.leftStick ().whileTrue(defer { Drive.driveToPoint(FieldManager.closestAlignPoint(Drive.localizer.pose, FieldManager.Level.L4, FieldManager.ScoringSide.LEFT), { Drive.localizer.singleTagPose }) })
        driverController.rightStick ().whileTrue(defer { Drive.driveToPoint(FieldManager.closestAlignPoint(Drive.localizer.pose, FieldManager.Level.L4, FieldManager.ScoringSide.RIGHT), { Drive.localizer.singleTagPose})})
        driverController.povUp ().whileTrue(defer { Drive.driveToPoint(FieldManager.closestAlignPoint(Drive.localizer.pose, FieldManager.Level.L3, FieldManager.ScoringSide.LEFT), { Drive.localizer.singleTagPose})})
        driverController.povRight ().whileTrue(defer { Drive.driveToPoint(FieldManager.closestAlignPoint(Drive.localizer.pose, FieldManager.Level.L3, FieldManager.ScoringSide.RIGHT), { Drive.localizer.singleTagPose})})
        driverController.povLeft ().whileTrue(defer { Drive.driveToPoint(FieldManager.closestAlignPoint(Drive.localizer.pose, FieldManager.Level.L2, FieldManager.ScoringSide.LEFT), { Drive.localizer.singleTagPose})})
        driverController.povDown ().whileTrue(defer { Drive.driveToPoint(FieldManager.closestAlignPoint(Drive.localizer.pose, FieldManager.Level.L2, FieldManager.ScoringSide.RIGHT), { Drive.localizer.singleTagPose})})

        // Switch to X pattern when X button is pressed
        driverController.x().onTrue(Commands.runOnce({ Drive.xPose() }, Drive))

//        driverController.povUp().onTrue(Commands.runOnce({ Armavator.setElevatorPercentOut(0.1) }))
//        driverController.povUp().onFalse(Commands.runOnce({ Armavator.setElevatorPercentOut(0.0) }))
//        driverController.povDown().onTrue(Commands.runOnce({ Armavator.setElevatorPercentOut(-0.1) }))
//        driverController.povDown().onFalse(Commands.runOnce({ Armavator.setElevatorPercentOut(0.0) }))

        // Reset gyro to 0° when B button is pressed
        driverController.back().onTrue(
            runOnceCommand(Drive) {
                println("zero gyro")
                Drive.zeroGyro()
            }.ignoringDisable(true))

        // Reset position to zero
        driverController.start().onTrue(
            runOnceCommand(Drive) {
                Drive.pose = Pose2d(Translation2d(3.0, 3.0), Drive.heading)
            }.ignoringDisable(true))
    }

    override fun periodic() {
        LoopLogger.record("b4 OI piodc")
        driverNotConnectedAlert.set(driverDebouncer.calculate(driverController.isConnected))
        operatorNotConnectedAlert.set(operatorDebouncer.calculate(operatorController.isConnected))
        LoopLogger.record("OI piodc")
    }

    /**
     * Uses the distance formula to remove the 90 degree "snap" that Xbox joystick axes do at extreme magnitudes.
     * Also desaturates (prevents magnitudes over 1).
     */
    fun unsnapAndDesaturateJoystick(rawX: Double, rawY: Double): Pair<Double, Double> {
        return if (hypot(rawX, rawY) > 1.0) {
            //magnitude is > 1, something is being "snapped" or the value is inaccurate
            if (rawX.absoluteValue >= 1.0) {
                //x not trustworthy
                Pair(sqrt(1 - rawY.square()).withSign(rawX), rawY)
            } else if (rawY.absoluteValue >= 1.0) {
                //y not trustworthy
                Pair(rawX, sqrt(1 - rawX.square()).withSign(rawY))
            } else {
                //Both axes are not snapping, but the magnitude > 1 meaning that one or both axes are inaccurate. (Joystick is drifting or warping)
                //We will assume the joystick magnitude equals 1 and assume both axes are untrustworthy.
                //To get the correct angle, we will perform a weighted average between the raw and calculated axes.
                //Larger raw axis values are untrustworthy. A raw axis value is used as the weight for the calculated axis provided by the opposing (perpendicular) axis.
                //In the weighted average: rawX is used as the weight for xCalc (xCalc was calculated using rawY)

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
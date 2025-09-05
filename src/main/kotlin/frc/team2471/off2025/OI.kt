package frc.team2471.off2025

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.FieldManager.onOpposingAllianceSide
import frc.team2471.off2025.FieldManager.reflectAcrossField
import frc.team2471.off2025.util.control.LoopLogger
import frc.team2471.off2025.util.control.MeanCommandXboxController
import frc.team2471.off2025.util.control.runCommand
import frc.team2471.off2025.util.control.toCommand
import frc.team2471.off2025.util.math.deadband
import frc.team2471.off2025.util.math.normalize
import frc.team2471.off2025.util.units.asRotation2d
import frc.team2471.off2025.util.units.degrees
import kotlin.math.absoluteValue

object OI: SubsystemBase("OI") {
    val driverController = MeanCommandXboxController(0, false)
    val operatorController = MeanCommandXboxController(1)

    val deadbandDriver = 0.08
    val deadbandOperator = 0.1

    val driveTranslationX: Double
        get() = driverController.leftY.deadband(deadbandDriver)

    val driveTranslationY: Double
        get() = driverController.leftX.deadband(deadbandDriver)

    val rawDriveTranslation: Translation2d
        get() {
            val translation = Translation2d(driveTranslationX, driveTranslationY)
            return if (translation.norm > 1.0) {
                translation.normalize()
            } else {
                translation
            }
        }

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


        driverController.a().onTrue(::goToDrivePose.toCommand(Armavator))
//        driverController.b().onTrue(runOnceCommand(Armavator){ Armavator.goToPose(Pose.SCORE_L3)})
//        driverController.b().onTrue(runOnceCommand(Armavator){ Armavator.goToPose(Pose.DRIVE_PIVOT_ONE_THIRD_TEST)})
//        driverController.y().onTrue(runOnceCommand(Armavator){ Armavator.goToPose(Pose.DRIVE_PIVOT_TWO_THIRDS_TEST)})
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
        driverController.b().whileTrue(coralStationIntake())
//        driverController.x().whileTrue(defer { ampAlign() })
        driverController.x().whileTrue(defer { algaeDescore() })

/*        driverController.a().onTrue(runOnce {
            Drive.questSimConnected = !Drive.questSimConnected
            println("questSimConnected = ${Drive.questSimConnected}")
        })*/

        driverController.rightTrigger(0.9).whileTrue(runCommand { Intake.score() })

        driverController.leftBumper().whileTrue(groundIntake(false))
        driverController.rightBumper().whileTrue(groundIntake(true))

        driverController.leftStick ().whileTrue(defer { alignToScore(FieldManager.Level.L4, FieldManager.ScoringSide.LEFT) })
        driverController.rightStick ().whileTrue(defer { alignToScore(FieldManager.Level.L4, FieldManager.ScoringSide.RIGHT) })

        driverController.povUp ().whileTrue(defer { alignToScore(FieldManager.Level.L3, FieldManager.ScoringSide.LEFT) })
        driverController.povRight ().whileTrue(defer { alignToScore(FieldManager.Level.L3, FieldManager.ScoringSide.RIGHT) })
        driverController.povLeft ().whileTrue(defer { alignToScore(FieldManager.Level.L2, FieldManager.ScoringSide.LEFT) })
        driverController.povDown ().whileTrue(defer { alignToScore(FieldManager.Level.L2, FieldManager.ScoringSide.RIGHT) })

        // Switch to X pattern when X button is pressed
    //    driverController.x().onTrue(Commands.runOnce({ Drive.xPose() }, Drive))

        // Reset gyro to 0° when B button is pressed
        driverController.back().onTrue({
                println("zero gyro")
                Drive.zeroGyro()
            }.toCommand(Drive).ignoringDisable(true))

        // Reset position to zero
        driverController.start().onTrue( {
            Drive.pose = Pose2d(Translation2d(3.0, 3.0), Drive.heading)
        }.toCommand(Drive).ignoringDisable(true))
    }

    override fun periodic() {
        LoopLogger.record("b4 OI piodc")
        driverNotConnectedAlert.set(driverDebouncer.calculate(driverController.isConnected))
        operatorNotConnectedAlert.set(operatorDebouncer.calculate(operatorController.isConnected))
        LoopLogger.record("OI piodc")
    }
}
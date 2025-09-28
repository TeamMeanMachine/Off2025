package frc.team2471.off2025

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.control.LoopLogger
import frc.team2471.off2025.util.control.MeanCommandXboxController
import frc.team2471.off2025.util.control.commands.finallyRun
import frc.team2471.off2025.util.control.commands.runCommand
import frc.team2471.off2025.util.control.commands.toCommand
import frc.team2471.off2025.util.math.deadband
import frc.team2471.off2025.util.math.normalize

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
        val coralMode = { driverController.leftTriggerAxis < 0.1 }
        val algaeMode = { driverController.leftTriggerAxis > 0.8 }
        // Drive Pose
        driverController.a().onTrue(runOnce { goToDrivePose() })
        // L1
        driverController.y().and(coralMode).whileTrue(defer { alignToScore(FieldManager.Level.L1, null) })
        // Barge
        driverController.y().and (algaeMode).whileTrue(defer { bargeAlignAndScore() })
        // Processor Align
        driverController.b().and (algaeMode).whileTrue(defer { ampAlign() })
        // Coral Station Intake
        driverController.b().and (coralMode).whileTrue(coralStationIntake())
        // Algae Descore
        driverController.x().and (coralMode).whileTrue(defer { algaeDescore() })
        // Climb heading align
        driverController.x().and (algaeMode).whileTrue( runOnce { println("climb heading align and deploy, does not do anything yet") })


        // Coral Ground Intake
        driverController.leftBumper().and (coralMode).whileTrue(groundIntake(false))
        driverController.rightBumper().and (coralMode).whileTrue(groundIntake(true))

        // Climb
        (driverController.rightBumper().or(driverController.leftBumper())).and (algaeMode).whileTrue(runOnce { println("CLIMB, does not do anything yet") })






        // Score
        driverController.rightTrigger(0.9).whileTrue(
            runCommand { Intake.intakeState = IntakeState.SCORING }
                .finallyRun { Intake.intakeState = IntakeState.HOLDING })

        // Algae Ground Intake
        driverController.leftStick().and (algaeMode).onTrue(algaeGroundIntake(false))
        driverController.rightStick().and (algaeMode).onTrue(algaeGroundIntake(true))

        // L4
        driverController.leftStick ().and (coralMode).whileTrue(defer { alignToScoreWithDelayDistance(FieldManager.Level.L4, FieldManager.ScoringSide.LEFT) })
        driverController.rightStick ().and (coralMode).whileTrue(defer { alignToScoreWithDelayDistance(FieldManager.Level.L4, FieldManager.ScoringSide.RIGHT) })

        // L3-L2
        driverController.povUp ().whileTrue(defer { alignToScore(FieldManager.Level.L3, FieldManager.ScoringSide.LEFT) })
        driverController.povRight ().whileTrue(defer { alignToScore(FieldManager.Level.L3, FieldManager.ScoringSide.RIGHT) })
        driverController.povLeft ().whileTrue(defer { alignToScore(FieldManager.Level.L2, FieldManager.ScoringSide.LEFT) })
        driverController.povDown ().whileTrue(defer { alignToScore(FieldManager.Level.L2, FieldManager.ScoringSide.RIGHT) })

//        driverController.leftBumper().onTrue(defer { Vision.alignToGamepiece { !driverController.leftBumper().asBoolean } })

        // Zero Gyro
        driverController.back().onTrue({
                println("zero gyro")
                Drive.zeroGyro()
                Vision.gyroReset()
            }.toCommand(Drive).ignoringDisable(true))

        // Reset Odometry Position
        driverController.start().and(coralMode).onTrue( {
            Drive.pose = Pose2d(Translation2d(3.0, 3.0), Drive.heading)
        }.toCommand(Drive).ignoringDisable(true))

        driverController.start().and (algaeMode).onTrue(runOnce { Drive.resetOdometryToAbsolute() })
    }

    override fun periodic() {
        LoopLogger.record("b4 OI piodc")
        driverNotConnectedAlert.set(driverDebouncer.calculate(driverController.isConnected))
        operatorNotConnectedAlert.set(operatorDebouncer.calculate(operatorController.isConnected))
        LoopLogger.record("OI piodc")
    }
}
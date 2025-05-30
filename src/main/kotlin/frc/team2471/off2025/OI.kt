package frc.team2471.off2025

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team2471.off2025.commands.DriveCommands
import frc.team2471.off2025.subsystems.drive.Drive
import frc.team2471.off2025.util.cube
import frc.team2471.off2025.util.deadband
import frc.team2471.off2025.util.runOnceCommand
import frc.team2471.off2025.util.squareWithSign

object OI {
    val driverController = CommandXboxController(0)
    val operatorController = CommandXboxController(1)

    val deadbandDriver = 0.08
    val deadbandOperator = 0.1

    val driveTranslationX: Double
        get() = -driverController.leftY.deadband(deadbandDriver).squareWithSign()

    val driveTranslationY: Double
        get() = -driverController.leftX.deadband(deadbandDriver).squareWithSign()

    val driveRotation: Double
        get() = -driverController.rightX.deadband(deadbandDriver).cube()

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





    init {
        // Default command, normal field-relative drive
        Drive.defaultCommand = DriveCommands.joystickDrive()



        // Lock to 0° when A button is held
        driverController.a().whileTrue(DriveCommands.joystickDriveAtAngle({ Rotation2d() }))

        // Switch to X pattern when X button is pressed
        driverController.x().onTrue(Commands.runOnce({ Drive.xPose() }, Drive))

        // Reset gyro to 0° when B button is pressed
        driverController.back().onTrue(
            runOnceCommand({
                Drive.pose = Pose2d(Drive.pose.translation, Rotation2d())
                Drive.arcPose = Pose2d(Drive.arcPose.translation, Rotation2d())
                             },
            Drive
        ).ignoringDisable(true))

        // Reset position to zero
        driverController.start().onTrue(
            runOnceCommand({
                Drive.pose = Pose2d(Translation2d(), Drive.pose.rotation)
                Drive.arcPose = Pose2d(Translation2d(), Drive.arcPose.rotation)
                             },
            Drive
        ).ignoringDisable(true))
    }
}
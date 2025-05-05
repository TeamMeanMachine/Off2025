package frc.team2471.off2025

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team2471.off2025.commands.DriveCommands
import frc.team2471.off2025.subsystems.drive.Drive

object OI {
    // Controller
    private val driverController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)



    init {
        // Default command, normal field-relative drive
        Drive.defaultCommand = DriveCommands.joystickDrive(
            { -driverController.leftY },
            { -driverController.leftX },
            { -driverController.rightX })



        // Lock to 0° when A button is held
        driverController.a().whileTrue(
            DriveCommands.joystickDriveAtAngle(
                { -driverController.leftY },
                { -driverController.leftX },
                { Rotation2d() })
        )

        // Switch to X pattern when X button is pressed
        driverController.x().onTrue(Commands.runOnce({ Drive.xPose() }, Drive))

        // Reset gyro to 0° when B button is pressed
        driverController.back().onTrue(
            Commands.runOnce({
                Drive.pose = Pose2d(Drive.pose.translation, Rotation2d())
                Drive.odomPose = Pose2d(Drive.odomPose.translation, Rotation2d())
                             },
            Drive
        ).ignoringDisable(true))

        // Reset position to zero
        driverController.start().onTrue(
            Commands.runOnce({
                Drive.pose = Pose2d(Translation2d(), Drive.pose.rotation)
                Drive.odomPose = Pose2d(Translation2d(), Drive.odomPose.rotation)
                             },
            Drive
        ).ignoringDisable(true))
    }
}
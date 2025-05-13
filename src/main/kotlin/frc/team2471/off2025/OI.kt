package frc.team2471.off2025

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team2471.off2025.commands.DriveCommands
import frc.team2471.off2025.subsystems.IntakeSubsystem
import frc.team2471.off2025.subsystems.drive.Drive

object OI {
    // Controller
    private val driverController = CommandXboxController(0)



    init {

        //TODO: step 4: when driver presses a button set Intake power to 0.9
        //TODO: step 4: when driver presses b button set Intake power to 0.0


        //TODO: step 6: Use driver left and right triggers to set any power between -1.0 to 1.0. Set to 0.0 if not pressing triggers.




        //DRIVE TRIGGERS, FOR DRIVING REAL ROBOTS:

        // Default command, normal field-relative drive
        Drive.defaultCommand = DriveCommands.joystickDrive(
            { -driverController.leftY },
            { -driverController.leftX },
            { -driverController.rightX })



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
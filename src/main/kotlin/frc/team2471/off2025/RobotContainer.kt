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
package frc.team2471.off2025

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team2471.off2025.commands.DriveCommands
import frc.team2471.off2025.commands.DriveCommands.feedforwardCharacterization
import frc.team2471.off2025.commands.DriveCommands.wheelRadiusCharacterization
import frc.team2471.off2025.commands.ExampleCommand
import frc.team2471.off2025.subsystems.drive.Drive
import frc.team2471.off2025.util.degrees
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // Subsystems
    val allSubsystems = arrayOf(Drive)

    // Controller
    private val driverController = CommandXboxController(0)

    // Dashboard inputs
    private val autoChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("Auto Chooser", AutoBuilder.buildAutoChooser()).apply {
        addOption("ExampleCommand", ExampleCommand())
    }
    private val testChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("Test Chooser").apply {
        // Set up SysId routines
        addOption("Drive Wheel Radius Characterization", wheelRadiusCharacterization())
        addOption("Drive Simple FF Characterization", feedforwardCharacterization())
        addOption("Drive SysId (Quasistatic Forward)", Drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        addOption("Drive SysId (Quasistatic Reverse)", Drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        addOption("Drive SysId (Dynamic Forward)", Drive.sysIdDynamic(SysIdRoutine.Direction.kForward))
        addOption("Drive SysId (Dynamic Reverse)", Drive.sysIdDynamic(SysIdRoutine.Direction.kReverse))
        addOption("Zero Turn Encoders", Drive.zeroTurnEncoders())
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        allSubsystems.forEach { it }
        configureButtonBindings()

        println(360.0.degrees)
        //3.60E+2
        //360.0 degrees

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [CommandXboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {
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
        driverController.back().onTrue(Commands.runOnce({ Drive.pose = Pose2d(Drive.pose.translation, Rotation2d()) },
            Drive
        ).ignoringDisable(true))

        // Reset position to zero
        driverController.start().onTrue(Commands.runOnce({ Drive.pose = Pose2d(Translation2d(), Drive.pose.rotation) },
            Drive
        ).ignoringDisable(true))
    }

    val autonomousCommand: Command?
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = autoChooser.get()
    val testCommand: Command?
        get() = testChooser.get()
}

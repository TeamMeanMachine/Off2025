package frc.team2471.off2025

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team2471.off2025.Elevator

object OI {
    // Controller
    private val driverController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)



    init {

        //TODO: Use left and right triggers to run percent output

        //TODO: up Dpad -> move up 1 inch
        //TODO: down Dpad -> move down 1 inch

        //TODO: use motionMagic to move up/down 10 inches smoothly using left/right dPad











        println("Finished OI init")
    }
}
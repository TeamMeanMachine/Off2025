package frc.team2471.off2025.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.team2471.off2025.OI
import frc.team2471.off2025.subsystems.drive.Drive
import org.littletonrobotics.junction.Logger


fun joystickTest(): Command {
    val joystickValues = mutableListOf<Translation2d>()


    return Commands.run({
        val (x, y) = OI.unsnapAndDesaturateJoystick(OI.driveTranslationX, OI.driveTranslationY)
        val translation = Translation2d(x, y)
        if (translation.norm > 0.5) {
            if (Math.random() > 0.5) {
//                joystickValues.clear()
                joystickValues.add(translation)
            }
            Logger.recordOutput("joystickValues", *joystickValues.toTypedArray())
            Logger.recordOutput("rawJoystickValue", Translation2d(OI.driveTranslationX, OI.driveTranslationY))
        }
    }, Drive)
}
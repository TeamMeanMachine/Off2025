package frc.team2471.off2025

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.team2471.off2025.util.parallelCommand
import frc.team2471.off2025.util.runCommand
import frc.team2471.off2025.util.runOnce
import frc.team2471.off2025.util.sequenceCommand
import frc.team2471.off2025.util.units.asMeters
import frc.team2471.off2025.util.units.asRotation2d
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.inches
import frc.team2471.off2025.util.waitUntilCommand
import kotlin.text.compareTo

fun groundIntake(isFlipped: Boolean): Command {
    return runOnce(Armavator){
        Intake.scoreAlgae = false
        Armavator.goToPose(Pose.INTAKE_GROUND, isFlipped)
        Intake.intakeState = IntakeState.INTAKING
    }
}

fun goToDrivePose(): Command {
    return runOnce(Armavator ){
        Armavator.goToPose(Pose.DRIVE)
        Intake.intakeState = IntakeState.HOLDING
    }
}

fun ampAlign(): Command {
    val ampAlignPoint = FieldManager.ampAlignPoint(Drive.localizer.pose)
    return parallelCommand(
        Drive.driveToPoint(ampAlignPoint, { Drive.localizer.pose}),
        sequenceCommand(
                       waitUntilCommand{ ampAlignPoint.translation.getDistance(Drive.localizer.pose.translation) < 3.0.inches.asMeters },
            runCommand(Armavator ){
                Intake.scoreAlgae = true
                Armavator.goToPose(Pose.INTAKE_GROUND, Drive.heading.degrees > 0.0) }
        )
    )
}

fun coralStationIntake(): Command {
    return runCommand(Armavator, Drive) {
        val alignmentAngleAndFlipped = FieldManager.getHumanStationAlignHeading(Drive.localizer.pose)
        Drive.driveAtAngle(alignmentAngleAndFlipped.first.asRotation2d)
        Armavator.goToPose(Pose.INTAKE_CORAL_STATION, alignmentAngleAndFlipped.second)
        Intake.intakeState = IntakeState.INTAKING
    }.andThen(goToDrivePose())
}

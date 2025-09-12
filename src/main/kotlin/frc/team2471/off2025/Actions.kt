package frc.team2471.off2025

import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.off2025.util.control.finallyRun
import frc.team2471.off2025.util.control.parallelCommand
import frc.team2471.off2025.util.control.runCommand
import frc.team2471.off2025.util.control.sequenceCommand
import frc.team2471.off2025.util.units.asMeters
import frc.team2471.off2025.util.units.asRotation2d
import frc.team2471.off2025.util.units.inches
import frc.team2471.off2025.util.control.waitUntilCommand
import frc.team2471.off2025.util.units.feet

fun groundIntake(isFlipped: Boolean): Command {
    return runCommand(Armavator) {
//        println("going to ground intake")
        Intake.scoreAlgae = false
        Armavator.goToPose(Pose.INTAKE_GROUND, isFlipped, false)
        Intake.intakeState = IntakeState.INTAKING
    }.finallyRun { goToDrivePose() }
}

fun goToDrivePose() {
    println("going to drive pose")
    Armavator.goToPose(Pose.DRIVE, optimizePivot = false)
    Armavator.resetPivot()
    Intake.intakeState = IntakeState.HOLDING
}

fun ampAlign(): Command {
    val ampAlignPoint = FieldManager.ampAlignPoint(Drive.localizer.pose)
    return parallelCommand(
        Drive.driveToPoint(ampAlignPoint, { Drive.localizer.pose}),
        sequenceCommand(
            waitUntilCommand { ampAlignPoint.translation.getDistance(Drive.localizer.pose.translation) < 3.0.inches.asMeters },
            runCommand(Armavator ){
                Intake.scoreAlgae = true
                Armavator.goToPose(Pose.INTAKE_GROUND, Drive.heading.degrees > 0.0, false)
            }
        )
    )
}
fun alignToScore(level: FieldManager.Level, side: FieldManager.ScoringSide): Command {

    val closestAlignPose = FieldManager.closestAlignPoint(Drive.localizer.pose, level, side)

    return parallelCommand(
          Drive.driveToPoint(closestAlignPose.first, { Drive.localizer.singleTagPose}),
        runCommand(Armavator){
            val poseAndOptimize = when (level){
                FieldManager.Level.L1 -> Pose.SCORE_L1 to false
                FieldManager.Level.L2 -> Pose.SCORE_L2 to true
                FieldManager.Level.L3 -> Pose.SCORE_L3 to true
                FieldManager.Level.L4 -> Pose.SCORE_L4 to true
            }

            Armavator.goToPose(poseAndOptimize.first, closestAlignPose.second, poseAndOptimize.second)
        }
    )
}

fun coralStationIntake(): Command {
    return runCommand(Armavator, Drive) {
//        println("coral station intake")
        val alignmentAngleAndFlipped = FieldManager.getHumanStationAlignHeading(Drive.localizer.pose)
        Drive.driveAtAngle(alignmentAngleAndFlipped.first.asRotation2d)
        Armavator.goToPose(Pose.INTAKE_CORAL_STATION, alignmentAngleAndFlipped.second, false)
        Intake.intakeState = IntakeState.INTAKING
    }.finallyRun { goToDrivePose() }
}

fun algaeDescore(): Command {
    val alignPoseAndLevel = FieldManager.getClosestReefAlgae(Drive.localizer.pose)
    return parallelCommand(
        Drive.driveToPoint(alignPoseAndLevel.first, { Drive.localizer.singleTagPose }),
        sequenceCommand(
            waitUntilCommand { alignPoseAndLevel.first.translation.getDistance(Drive.localizer.singleTagPose.translation) < 2.0.feet.asMeters },
            runCommand(Armavator) {
                Intake.intakeState = IntakeState.REVERSING
                Armavator.goToPose(if (alignPoseAndLevel.second == FieldManager.AlgaeLevel.LOW) Pose.ALGAE_DESCORE_LOW else Pose.ALGAE_DESCORE_HIGH, alignPoseAndLevel.third, optimizePivot = false)
            }
        )
    ).finallyRun {
        goToDrivePose()
    }
}
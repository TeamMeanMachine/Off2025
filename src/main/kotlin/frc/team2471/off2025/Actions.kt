package frc.team2471.off2025

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.off2025.FieldManager.getApproachAngle
import frc.team2471.off2025.FieldManager.onOpposingAllianceSide
import frc.team2471.off2025.FieldManager.reflectAcrossField
import frc.team2471.off2025.util.control.finallyRun
import frc.team2471.off2025.util.control.leftStickButton
import frc.team2471.off2025.util.control.onlyRunWhileFalse
import frc.team2471.off2025.util.control.onlyRunWhileTrue
import frc.team2471.off2025.util.control.parallelCommand
import frc.team2471.off2025.util.control.rightStickButton
import frc.team2471.off2025.util.control.runCommand
import frc.team2471.off2025.util.control.runOnce
import frc.team2471.off2025.util.control.sequenceCommand
import frc.team2471.off2025.util.units.asMeters
import frc.team2471.off2025.util.units.asRotation2d
import frc.team2471.off2025.util.units.inches
import frc.team2471.off2025.util.control.waitUntilCommand
import frc.team2471.off2025.util.math.findClosestPointOnLine
import frc.team2471.off2025.util.units.absoluteValue
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.feet
import kotlin.math.absoluteValue

fun groundIntake(isFlipped: Boolean): Command {
    return runCommand(Armavator) {
//        println("going to ground intake")
        Intake.scoreAlgae = false
        Armavator.normalSpeed()
        Armavator.goToPose(Pose.INTAKE_GROUND, isFlipped, false)
        Intake.intakeState = IntakeState.INTAKING
    }.finallyRun { goToDrivePose() }
}

fun goToDrivePose(optionalPivot: Angle? = null) {
    println("going to drive pose")
    Armavator.goToPose(Pose.DRIVE.apply { pivotAngle = optionalPivot ?: pivotAngle }, optimizePivot = false)
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
                Armavator.slowSpeed()
                Armavator.goToPose(Pose.INTAKE_GROUND, Drive.heading.degrees > 0.0, false)
            }
        )
    ).finallyRun {
        Armavator.normalSpeed()
    }
}
fun alignToScore(level: FieldManager.Level, side: FieldManager.ScoringSide?): Command {

    val closestAlignPose = FieldManager.closestAlignPoint(Drive.localizer.pose, level, side)

    return parallelCommand(
          Drive.driveToAutopilotPoint(closestAlignPose.first, { Drive.localizer.singleTagPose}, { getApproachAngle(Drive.localizer.singleTagPose) }),
//          Drive.driveToPoint(closestAlignPose.first, { Drive.localizer.singleTagPose}),
        runCommand(Armavator){
            val poseAndOptimize = when (level){
                FieldManager.Level.L1 -> Pose.SCORE_L1 to false
                FieldManager.Level.L2 -> Pose.SCORE_L2 to true
                FieldManager.Level.L3 -> Pose.SCORE_L3 to true
                FieldManager.Level.L4 -> Pose.SCORE_L4 to true
            }

            Armavator.normalSpeed()
            Intake.scoreAlgae = false
            Armavator.goToPose(poseAndOptimize.first, closestAlignPose.second, poseAndOptimize.second)
        }
    )
}

fun alignToScoreWithDelayDistance(level: FieldManager.Level, side: FieldManager.ScoringSide?): Command {
    var closestAlignPose: Pair<Pose2d, Boolean>? = null
    val poseAndOptimize = when (level){
        FieldManager.Level.L1 -> Pose.SCORE_L1 to false
        FieldManager.Level.L2 -> Pose.SCORE_L2 to true
        FieldManager.Level.L3 -> Pose.SCORE_L3 to true
        FieldManager.Level.L4 -> Pose.SCORE_L4 to true
    }
    val delayDistanceAndIntermediate = when (level) {
        FieldManager.Level.L4 -> 40.0.inches to 20.0.degrees
        else -> 0.0.inches to null
    }

    return sequenceCommand(
        runOnce {
            closestAlignPose = FieldManager.closestAlignPoint(Drive.localizer.pose, level, side)
        },
        parallelCommand(
            Drive.driveToPoint({ closestAlignPose!!.first }, { Drive.localizer.singleTagPose }),
            Armavator.goToPose(poseAndOptimize.first, { closestAlignPose!!.second }, poseAndOptimize.second, delayDistanceAndIntermediate.first, delayDistanceAndIntermediate.second)
        )
    )
}

fun alignToScoreWithDelayDistance(alignPoint: () -> Pose2d, level: FieldManager.Level): Command {
    var isFlipped = FieldManager.closestAlignPoint(alignPoint(), level).second
    val poseAndOptimize = when (level){
        FieldManager.Level.L1 -> Pose.SCORE_L1 to false
        FieldManager.Level.L2 -> Pose.SCORE_L2 to true
        FieldManager.Level.L3 -> Pose.SCORE_L3 to true
        FieldManager.Level.L4 -> Pose.SCORE_L4 to true
    }
    val delayDistanceAndIntermediate = when (level) {
        FieldManager.Level.L4 -> 40.0.inches to 20.0.degrees
        else -> 0.0.inches to null
    }

    return sequenceCommand(
        runOnce {
            isFlipped = FieldManager.closestAlignPoint(alignPoint(), level).second
        },
        parallelCommand(
            Drive.driveToPoint({ if (isFlipped) Pose2d(alignPoint().translation, alignPoint().rotation.rotateBy(180.0.degrees.asRotation2d)) else alignPoint() }, { Drive.localizer.singleTagPose }),
            Armavator.goToPose(poseAndOptimize.first, { isFlipped }, poseAndOptimize.second, delayDistanceAndIntermediate.first, delayDistanceAndIntermediate.second)
        )
    )
}

fun coralStationIntake(): Command {
    return runCommand(Armavator, Drive) {
//        println("coral station intake")
        val alignmentAngleAndFlipped = FieldManager.getHumanStationAlignHeading(Drive.localizer.pose)
        Drive.driveAtAngle(alignmentAngleAndFlipped.first.asRotation2d)
        Armavator.normalSpeed()
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
                Intake.intakeState = IntakeState.ALGAE_GROUND
                Armavator.normalSpeed()
                Armavator.goToPose(if (alignPoseAndLevel.second == FieldManager.AlgaeLevel.LOW) Pose.ALGAE_DESCORE_LOW else Pose.ALGAE_DESCORE_HIGH, alignPoseAndLevel.third, optimizePivot = false)
            }
        )
    ).finallyRun {
        goToDrivePose()
    }
}

fun bargeAlignAndScore(): Command {
    val pointOne = FieldManager.bargeAlignPoints.first.reflectAcrossField { Drive.localizer.pose.onOpposingAllianceSide() }
    val pointTwo = FieldManager.bargeAlignPoints.second.reflectAcrossField { Drive.localizer.pose.onOpposingAllianceSide() }
    val isFlipped = Drive.heading.degrees.absoluteValue > 90.0
    val poseSupplier = { Drive.localizer.pose }
    return parallelCommand(
        Drive.joystickDriveAlongLine(pointOne, pointTwo, (if (isFlipped) 180.0 else 0.0).degrees.asRotation2d, poseSupplier),
        sequenceCommand(
            waitUntilCommand {
                Drive.localizer.pose.translation.getDistance(
                    findClosestPointOnLine(pointOne, pointTwo, poseSupplier().translation)
                ) < 3.0.feet.asMeters
            },
            runCommand(Armavator) {
                Intake.scoreAlgae = true
                Armavator.normalSpeed()
                Armavator.goToPose(Pose.BARGE_SCORE, isFlipped, optimizePivot = false)
            }
        )
    ).finallyRun {
        goToDrivePose(180.0.degrees)
        Intake.scoreAlgae = false
    }
}

fun algaeGroundIntake(isFlipped: Boolean): Command {
    return sequenceCommand(
        runOnce { // Spin up intake
            Intake.intakeState = IntakeState.ALGAE_GROUND
        },
        runCommand(Armavator) { // Go to intermediate position until pivot clears
            Armavator.goToPose(Pose.ALGAE_INTAKE_INTERMEDIATE, isFlipped, false)
        }.onlyRunWhileFalse {
            Armavator.noMovement || Armavator.pivotSetpointError.absoluteValue() < 20.0.degrees || !(OI.driverController.rightStickButton || OI.driverController.leftStickButton)
        },
        runCommand(Armavator) { // Go to intake ground position until bumpers are no longer pressed
            Armavator.goToPose(Pose.ALGAE_INTAKE_GROUND, isFlipped, false)
        }.onlyRunWhileTrue {
            OI.driverController.rightStickButton || OI.driverController.leftStickButton
        },
        runCommand(Armavator) { // Go to intermediate position until pivot clears
            Armavator.slowSpeed()
            Armavator.goToPose(Pose.ALGAE_INTAKE_INTERMEDIATE, isFlipped, false)
        }.onlyRunWhileFalse {
            Armavator.noMovement || Armavator.pivotSetpointError.absoluteValue() < 20.0.degrees
        }
    ).finallyRun { // End at drive pose
        goToDrivePose()
    }
}
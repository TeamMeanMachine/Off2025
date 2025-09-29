package frc.team2471.off2025

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.control.commands.finallyRun
import frc.team2471.off2025.util.control.commands.runCommand
import frc.team2471.off2025.util.units.asDegrees
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.radians
import frc.team2471.off2025.util.vision.limelight.LimelightMode
import frc.team2471.off2025.util.vision.limelight.VisionIO
import frc.team2471.off2025.util.vision.limelight.VisionIOLimelight
import org.littletonrobotics.junction.Logger
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.sign

object Vision : SubsystemBase() {
    val io: Array<VisionIO> = arrayOf(
        VisionIOLimelight("limelight-test") { Drive.heading.measure }
    )
    
    val inputs = arrayOf(
        VisionIO.VisionIOInputs()
    )

    var mode: LimelightMode = LimelightMode.GAMEPIECE

    override fun periodic() {
        for (i in io.indices) {
            if (io[i].mode != mode) io[i].mode = mode

            io[i].updateInputs(inputs[i])

            if (inputs[i].aprilTagPoseEstimate != Pose2d()) {
                Drive.addVisionMeasurement(inputs[i].aprilTagPoseEstimate, inputs[i].aprilTagTimestamp, VecBuilder.fill(0.01, 0.01, 1000000000.0))
            }

            if (io[i].mode == LimelightMode.GAMEPIECE) {
                val targetDimensions = inputs[i].targetDimensions

                Logger.recordOutput("Vision/target width", targetDimensions.first)
                Logger.recordOutput("Vision/target height", targetDimensions.second)
                Logger.recordOutput("Vision/wh ratio", targetDimensions.first / targetDimensions.second)

                if (inputs[i].hasTargets) {
                    Logger.recordOutput("Vision/Raw Coral Angle", getCoralAngle(inputs[0].targetCorners, inputs[i].targetCenter))
                }
            }

        }
    }

    fun gyroReset() {
        io.forEach { it.gyroReset() }
    }

    // exit supplier needs to return true when the command should end
    fun alignToGamepiece(exitSupplier: () -> Boolean): Command {

        //todo tune this
        val alignPIDController = PIDController(0.10, 0.0, 0.01)
        val rotatePIDController = PIDController(0.01, 0.0, 0.0)

        val offsetFilter = MedianFilter(5)
        val angleFilter = MedianFilter(11)

        return runCommand(Drive) {
            mode = LimelightMode.GAMEPIECE

            val targetDimensions = inputs[0].targetDimensions
            if (targetDimensions.second != 0.0) {
                val whRatio = targetDimensions.first / targetDimensions.second

                var translationChassisSpeeds = ChassisSpeeds()
                var rotationChassisSpeeds = ChassisSpeeds()
//                if (whRatio >= 1.2) {

                    val offset = offsetFilter.calculate(-inputs[0].targetCoords.first())

                    val pidOutput = alignPIDController.calculate(offset, 0.0)

                    val offsetHeading = Drive.heading + Rotation2d(90.0.degrees)

                    translationChassisSpeeds = ChassisSpeeds(pidOutput * offsetHeading.cos, pidOutput * offsetHeading.sin, 0.0)
//                }

                val angle = angleFilter.calculate(getCoralAngle(inputs[0].targetCorners, inputs[0].targetCenter).asDegrees).degrees
                Logger.recordOutput("Vision/Filtered Coral Angle", angle)

                if (angle.asDegrees.absoluteValue > 30) {
                    val pidOutput = rotatePIDController.calculate(angle.asDegrees, 0.0)

                    rotationChassisSpeeds = ChassisSpeeds(0.0, 0.0, pidOutput)
                }

                val chassisSpeeds = Drive.getChassisSpeedsFromJoystick().plus(translationChassisSpeeds).plus(rotationChassisSpeeds)

                Drive.driveVelocity(chassisSpeeds)
            }
        }.until {
            exitSupplier()
        }.finallyRun {
            mode = LimelightMode.APRILTAG
        }
    }


    fun onEnable() = io.forEach { it.enable() }

    fun onDisable() = io.forEach { it.disable() }


    fun getCoralAngle(corners: DoubleArray, center: Translation2d): Angle {
        if (corners.isEmpty()) {
            return 0.0.degrees
        }

        val cornersX = corners.filterIndexed { index, _ -> index % 2 == 0 }
        val cornersY = corners.filterIndexed { index, _ -> index % 2 == 1 }

        val cornersTranslation = cornersX.mapIndexed { index, value -> Translation2d(value, cornersY[index]) }.sortedBy { it.x }


        val dists = DoubleArray(cornersTranslation.size - 1) {0.0}

        for (i in 1..(cornersTranslation.size - 1)) {
            dists[i - 1] = cornersTranslation[0].getDistance(cornersTranslation[i])
        }

        val distThreshold = dists.max() * 0.6

        val group1: MutableList<Translation2d> = mutableListOf(cornersTranslation[0])
        val group2: MutableList<Translation2d> = mutableListOf()

        for (i in 1..cornersTranslation.size - 1) {
            if (dists[i - 1] > distThreshold) {
                group2.add(cornersTranslation[i])
            } else {
                group1.add(cornersTranslation[i])
            }
        }

        Logger.recordOutput("Vision/Group 1", *group1.toTypedArray())
        Logger.recordOutput("Vision/Group 2", *group2.toTypedArray())


        val group1Center = Translation2d(group1.map {it.x}.average(), group1.map {it.y}.average())
        val group2Center = Translation2d(group2.map {it.x}.average(), group2.map {it.y}.average())

        return atan2(group1Center.y - group2Center.y, group1Center.x - group2Center.x).radians.asDegrees.let {
            (if (it.absoluteValue > 90.0) it - it.sign * 180.0 else it).degrees
        }
    }
}
package frc.team2471.off2025

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.degrees
import frc.team2471.off2025.util.finallyRun
import frc.team2471.off2025.util.isBlueAlliance
import frc.team2471.off2025.util.runCommand
import frc.team2471.off2025.util.vision.LimelightMode
import frc.team2471.off2025.util.vision.VisionIO
import frc.team2471.off2025.util.vision.VisionIOLimelight
import org.littletonrobotics.junction.Logger

object Vision : SubsystemBase() {
    val io: Array<VisionIO> = arrayOf(
        VisionIOLimelight("limelight-test") { Drive.heading.measure }
    )
    
    val inputs = arrayOf(
        VisionIO.VisionIOInputs()
    )

    var mode: LimelightMode = LimelightMode.APRILTAG

    override fun periodic() {
        for (i in io.indices) {
            if (io[i].mode != mode) io[i].mode = mode

            io[i].updateInputs(inputs[i])

            if (inputs[i].aprilTagPoseEstimate != Pose2d()) {
                Drive.addVisionMeasurement(inputs[i].aprilTagPoseEstimate, inputs[i].aprilTagTimestamp, VecBuilder.fill(0.01, 0.01, 1000000000.0))
            }

            if (io[i].mode == LimelightMode.GAMEPIECE) {
                val targetDimensions = inputs[i].targetDimensions

                Logger.recordOutput("target width", targetDimensions.first)
                Logger.recordOutput("target height", targetDimensions.second)
                Logger.recordOutput("wh ratio", targetDimensions.first / targetDimensions.second)
            }
        }
    }

    fun gyroReset() {
        io.forEach { it.gyroReset() }
    }

    fun alignToGamepiece(): Command {
        return runCommand(Drive) {
            mode = LimelightMode.GAMEPIECE



            val offset = inputs[0].targetCoords.first() / 20

            val offsetHeading = Drive.heading + Rotation2d(90.0.degrees)

            val chassisSpeeds = Drive.getChassisSpeedsFromJoystick().apply {
                if (isBlueAlliance) {
                    vxMetersPerSecond *= -1.0
                    vyMetersPerSecond *= -1.0
                }
            }.plus(ChassisSpeeds(offset * offsetHeading.cos, offset * offsetHeading.sin, 0.0))

            Drive.driveVelocity(chassisSpeeds)
        }.finallyRun {
            mode = LimelightMode.APRILTAG
        }
    }
}
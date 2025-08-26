package frc.team2471.off2025

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.Timer.getFPGATimestamp
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.vision.VisionIO
import frc.team2471.off2025.util.vision.VisionIOLimelight

object Vision : SubsystemBase() {
    val io: Array<VisionIO> = arrayOf(
        VisionIOLimelight("limelight-test") { Drive.heading.measure }
    )
    
    val inputs = arrayOf(
        VisionIO.VisionIOInputs()
    )

    override fun periodic() {
        for (i in io.indices) {
            io[i].updateInputs(inputs[i])
            if (inputs[i].latestPoseEstimate != Pose2d()) {
                Drive.addVisionMeasurement(inputs[i].latestPoseEstimate, inputs[i].latestTimestamp, VecBuilder.fill(0.01, 0.01, 1000000000.0))
            }
        }
    }

    fun gyroReset() {
        io.forEach { it.gyroReset() }
    }
}
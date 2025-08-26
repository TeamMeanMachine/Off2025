package frc.team2471.off2025.util.vision

import com.ctre.phoenix6.Utils
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.measure.Angle
import frc.team2471.off2025.Robot
import frc.team2471.off2025.util.asDegrees
import frc.team2471.off2025.util.vision.LimelightHelpers
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs


class VisionIOLimelight(val name: String, val headingSupplier: () -> Angle): VisionIO {
    override fun updateInputs(inputs: VisionIO.VisionIOInputs) {
//        if (Robot.isDisabled) {
//            LimelightHelpers.SetIMUMode(name, 1)
            LimelightHelpers.SetRobotOrientation(name, headingSupplier.invoke().asDegrees, 0.0, 0.0, 0.0, 0.0, 0.0)
//        }

        val llPoseEstimate = if (Robot.isDisabled) LimelightHelpers.getBotPoseEstimate_wpiBlue(name) else LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)

        inputs.latestPoseEstimate = llPoseEstimate?.pose ?: Pose2d()
        inputs.latestTimestamp = Utils.fpgaToCurrentTime(llPoseEstimate?.timestampSeconds ?: 0.0)

        Logger.recordOutput("Limelight thing", inputs.latestPoseEstimate)
    }

    override fun enable() {
//        LimelightHelpers.SetIMUMode(name, 2)
    }

    override fun gyroReset() {
        LimelightHelpers.SetIMUMode(name, 1)
        LimelightHelpers.SetRobotOrientation(name, headingSupplier.invoke().asDegrees, 0.0, 0.0, 0.0, 0.0, 0.0)
//        LimelightHelpers.SetIMUMode(name, 2)
    }

}

interface VisionIO {
    fun updateInputs(inputs: VisionIOInputs)
    fun enable()
    fun gyroReset()

    open class VisionIOInputs : LoggableInputs {

        var latestPoseEstimate = Pose2d()
        var latestTimestamp = 0.0

        override fun toLog(table: LogTable) {
            table.put("Latest Pose Estimate", latestPoseEstimate)
            table.put("Latest Timestamp", latestTimestamp)
        }

        override fun fromLog(table: LogTable) {
            latestPoseEstimate = table.get("Latest Pose Estimate", latestPoseEstimate).first()
            latestTimestamp = table.get("Latest Timestamp", latestTimestamp)
        }
    }
}
package frc.team2471.off2025.util.vision

import com.ctre.phoenix6.Utils
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import frc.team2471.off2025.Robot
import frc.team2471.off2025.util.asDegrees
import frc.team2471.off2025.util.vision.LimelightHelpers
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs


class VisionIOLimelight(val name: String, val headingSupplier: () -> Angle): VisionIO {

    val heartbeatSub = NetworkTableInstance.getDefault().getTable(name).getDoubleTopic("hb").subscribe(0.0)
    var prevHeartbeats = MutableList(3) { 0.0 }

    override fun updateInputs(inputs: VisionIO.VisionIOInputs) {

        val heartbeat = heartbeatSub.get()
        inputs.isConnected = heartbeat != 0.0 && prevHeartbeats[2] != heartbeat
        prevHeartbeats.add(0, heartbeat)
        prevHeartbeats.removeAt(prevHeartbeats.size - 1)

//        if (Robot.isDisabled) {
//            LimelightHelpers.SetIMUMode(name, 1)
            LimelightHelpers.SetRobotOrientation(name, headingSupplier.invoke().asDegrees, 0.0, 0.0, 0.0, 0.0, 0.0)
//        }

        val llPoseEstimate = if (Robot.beforeFirstEnable) LimelightHelpers.getBotPoseEstimate_wpiBlue(name) else LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)

        inputs.latestPoseEstimate = llPoseEstimate?.pose ?: Pose2d()
        inputs.latestTimestamp = Utils.fpgaToCurrentTime(llPoseEstimate?.timestampSeconds ?: 0.0)

        Logger.processInputs(name, inputs)
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

        var isConnected = false
        var latestPoseEstimate = Pose2d()
        var latestStdDev = 0.0
        var latestTimestamp = 0.0

        override fun toLog(table: LogTable) {
            table.put("Is Connected", isConnected)
            table.put("Latest Pose Estimate", latestPoseEstimate)
            table.put("Latest Timestamp", latestTimestamp)
            table.put("Latest StdDev", latestStdDev)
        }

        override fun fromLog(table: LogTable) {
            isConnected = table.get("Is Connected", isConnected)
            latestPoseEstimate = table.get("Latest Pose Estimate", latestPoseEstimate).first()
            latestTimestamp = table.get("Latest Timestamp", latestTimestamp)
            latestStdDev = table.get("Latest StdDev", latestStdDev)
        }
    }
}
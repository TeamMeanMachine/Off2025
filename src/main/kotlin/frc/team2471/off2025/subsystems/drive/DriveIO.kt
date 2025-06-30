package frc.team2471.off2025.subsystems.drive

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.measure.Angle
import frc.team2471.off2025.util.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface DriveIO {
    fun updateInputs(inputs: DriveIOInputs)

    fun setDriveRequest(request: SwerveRequest)

    fun resetPose(pose: Pose2d?)

    fun resetHeading(angle: Angle? = null)

    fun resetPosition(translation: Translation2d?)

    fun updateSim()

    fun brakeMode()
    fun coastMode()

    fun setAngleOffsets()


    open class DriveIOInputs: LoggableInputs {
        @JvmField var pose = Pose2d()
        @JvmField var speeds = ChassisSpeeds()
        @JvmField var moduleStates = arrayOf<SwerveModuleState>()
        @JvmField var moduleTargets = arrayOf<SwerveModuleState>()
        @JvmField var modulePositions = arrayOf<SwerveModulePosition>()
        @JvmField var rawHeading = Rotation2d()
        @JvmField var timestamp = 0.0
        @JvmField var odometryPeriod = 0.0
        @JvmField var successfulDaqs = 0
        @JvmField var failedDaqs = 0

        @JvmField var moduleInputs: Array<ModuleInput> = Array(4) { ModuleInput() }
        @JvmField var gyroInputs = GyroInput()

        override fun toLog(table: LogTable) {
            //Swerve
            table.put("Pose", pose)
            table.put("Speeds", speeds)
            table.put("ModuleStates", *moduleStates)
            table.put("ModuleTargets", *moduleTargets)
            table.put("ModulePositions", *modulePositions)
            table.put("RawHeading", rawHeading)
            table.put("Timestamp", timestamp)
            table.put("OdometryPeriod", odometryPeriod)
            table.put("SuccessfulDaqs", successfulDaqs)
            table.put("FailedDaqs", failedDaqs)

            //modules
            moduleInputs.forEachIndexed { i, input ->
                val prefix = "Module $i/"
                table.put(prefix + "driveConnected", input.driveConnected)
                table.put(prefix + "driveSupplyCurrentAmps", input.driveSupplyCurrentAmps)
                table.put(prefix + "driveStatorCurrentAmps", input.driveStatorCurrentAmps)
                table.put(prefix + "driveAppliedVolts", input.driveAppliedVolts)
                table.put(prefix + "driveTemp", input.driveTemp)
                table.put(prefix + "steerConnected", input.steerConnected)
                table.put(prefix + "steerSupplyCurrentAmps", input.steerSupplyCurrentAmps)
                table.put(prefix + "steerStatorCurrentAmps", input.steerStatorCurrentAmps)
                table.put(prefix + "steerAppliedVolts", input.steerAppliedVolts)
                table.put(prefix + "steerTemp", input.steerTemp)
                table.put(prefix + "encoderConnected", input.encoderConnected)
                table.put(prefix + "encoderAbsoluteAngle", input.encoderAbsoluteAngle)
            }

            //gyro
            table.put("Gyro/gyroConnected", gyroInputs.gyroConnected)
            table.put("Gyro/yaw", gyroInputs.yaw)
            table.put("Gyro/yawRate", gyroInputs.yawRate)
            table.put("Gyro/pitch", gyroInputs.pitch)
            table.put("Gyro/pitchRate", gyroInputs.pitchRate)
            table.put("Gyro/roll", gyroInputs.roll)
            table.put("Gyro/rollRate", gyroInputs.rollRate)
            table.put("Gyro/xAccel", gyroInputs.xAccel)
            table.put("Gyro/yAccel", gyroInputs.yAccel)

            LoopLogger.record("DriveIOInputs.toLog()")
        }

        override fun fromLog(table: LogTable) {
            pose = table["Pose", pose].first()
            speeds = table["Speeds", speeds].first()
            moduleStates = table.get("ModuleStates", *moduleStates)
            moduleTargets = table.get("ModuleTargets", *moduleTargets)
            modulePositions = table.get("ModulePositions", *modulePositions)
            rawHeading = table["RawHeading", rawHeading].first()
            timestamp = table["Timestamp", timestamp]
            odometryPeriod = table["OdometryPeriod", odometryPeriod]
            successfulDaqs = table["SuccessfulDaqs", successfulDaqs]
            failedDaqs = table["FailedDaqs", failedDaqs]
            moduleInputs = Array(4) {
                val prefix = "Module $it/"
                ModuleInput().apply {
                    driveConnected = table[prefix + "driveConnected", driveConnected]
                    driveSupplyCurrentAmps = table[prefix + "driveSupplyCurrentAmps", driveSupplyCurrentAmps]
                    driveStatorCurrentAmps = table[prefix + "driveStatorCurrentAmps", driveStatorCurrentAmps]
                    driveAppliedVolts = table[prefix + "driveAppliedVolts", driveAppliedVolts]
                    driveTemp = table[prefix + "driveTemp", driveTemp]
                    steerConnected = table[prefix + "steerConnected", steerConnected]
                    steerSupplyCurrentAmps = table[prefix + "steerSupplyCurrentAmps", steerSupplyCurrentAmps]
                    steerStatorCurrentAmps = table[prefix + "steerStatorCurrentAmps", steerStatorCurrentAmps]
                    steerAppliedVolts = table[prefix + "steerAppliedVolts", steerAppliedVolts]
                    steerTemp = table[prefix + "steerTemp", steerTemp]
                    encoderConnected = table[prefix + "encoderConnected", encoderConnected]
                    encoderAbsoluteAngle = table[prefix + "encoderAbsoluteAngle", encoderAbsoluteAngle]
                }
            }
            gyroInputs = GyroInput().apply {
                gyroConnected = table["Gyro/gyroConnected", gyroConnected]
                yaw = table["Gyro/yaw", yaw]
                yawRate = table["Gyro/yawRate", yawRate]
                pitch = table["Gyro/pitch", pitch]
                pitchRate = table["Gyro/pitchRate", pitchRate]
                roll = table["Gyro/roll", roll]
                rollRate = table["Gyro/rollRate", rollRate]
                xAccel = table["Gyro/xAccel", xAccel]
                yAccel = table["Gyro/yAccel", yAccel]
            }
        }
    }

    class ModuleInput {
        var driveConnected = false
        var driveSupplyCurrentAmps = 0.0
        var driveStatorCurrentAmps = 0.0
        var driveAppliedVolts = 0.0
        var driveTemp = 0.0

        var steerConnected = false
        var steerSupplyCurrentAmps = 0.0
        var steerStatorCurrentAmps = 0.0
        var steerAppliedVolts = 0.0
        var steerTemp = 0.0

        var encoderConnected = false
        var encoderAbsoluteAngle = 0.0.degrees
    }

    class GyroInput {
        var gyroConnected = false
        var yaw = 0.0.degrees
        var yawRate = 0.0.degrees.perSecond
        var pitch = 0.0.degrees
        var pitchRate = 0.0.degrees.perSecond
        var roll = 0.0.degrees
        var rollRate = 0.0.degrees.perSecond
        var xAccel = 0.0.feet.perSecondPerSecond
        var yAccel = 0.0.feet.perSecondPerSecond
    }
}
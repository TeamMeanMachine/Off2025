package frc.team2471.off2025.subsystems.drive

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearAcceleration
import frc.team2471.off2025.util.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface DriveIO {
    fun updateInputs(inputs: DriveIOInputs)

    fun setDriveRequest(request: SwerveRequest)

    fun resetPose(pose: Pose2d?)

    fun resetHeading(angle: Angle)

    fun resetPosition(translation: Translation2d?)

    fun updateSim()

    fun brakeMode()
    fun coastMode()

    fun poseAt(timestampSeconds: Double): Pose2d?

    fun setAngleOffsets()


    open class DriveIOInputs: LoggableInputs {
        @JvmField var pose = Pose2d()
        @JvmField var speeds = ChassisSpeeds()
        @JvmField var moduleStates = Array(4) { SwerveModuleState() }
        @JvmField var moduleTargets = Array(4) { SwerveModuleState() }
        @JvmField var modulePositions = Array(4) { SwerveModulePosition() }
        @JvmField var rawHeading = Rotation2d()
        @JvmField var timestamp = 0.0
        @JvmField var odometryPeriod = 0.0
        @JvmField var successfulDaqs = 0
        @JvmField var failedDaqs = 0

        @JvmField var moduleInputs: Array<ModuleInput> = Array(4) { ModuleInput.Empty }
        @JvmField var gyroInputs = GyroInput.Empty

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
                table.put(prefix + "driveVelocity", input.driveVelocity)
                table.put(prefix + "driveAccel", input.driveAccel)
                table.put(prefix + "driveSupplyCurrentAmps", input.driveSupplyCurrentAmps)
                table.put(prefix + "driveStatorCurrentAmps", input.driveStatorCurrentAmps)
                table.put(prefix + "driveAppliedVolts", input.driveAppliedVolts)
                table.put(prefix + "driveTemp", input.driveTemp)
                table.put(prefix + "steerConnected", input.steerConnected)
                table.put(prefix + "steerVelocity", input.steerVelocity)
                table.put(prefix + "steerAccel", input.steerAccel)
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
                val m = moduleInputs[it]
                val prefix = "Module $it/"
                ModuleInput(
                    driveConnected = table[prefix + "driveConnected", m.driveConnected],
                    driveVelocity = table[prefix + "driveVelocityRot", m.driveVelocity],
                    driveAccel = table[prefix + "driveAccelRot", m.driveAccel],
                    driveSupplyCurrentAmps = table[prefix + "driveSupplyCurrentAmps", m.driveSupplyCurrentAmps],
                    driveStatorCurrentAmps = table[prefix + "driveStatorCurrentAmps", m.driveStatorCurrentAmps],
                    driveAppliedVolts = table[prefix + "driveAppliedVolts", m.driveAppliedVolts],
                    driveTemp = table[prefix + "driveTemp", m.driveTemp],
                    steerConnected = table[prefix + "steerConnected", m.steerConnected],
                    steerVelocity = table[prefix + "steerVelocityRot", m.steerVelocity],
                    steerAccel = table[prefix + "steerAccelRot", m.steerAccel],
                    steerSupplyCurrentAmps = table[prefix + "steerSupplyCurrentAmps", m.steerSupplyCurrentAmps],
                    steerStatorCurrentAmps = table[prefix + "steerStatorCurrentAmps", m.steerStatorCurrentAmps],
                    steerAppliedVolts = table[prefix + "steerAppliedVolts", m.steerAppliedVolts],
                    steerTemp = table[prefix + "steerTemp", m.steerTemp],
                    encoderConnected = table[prefix + "encoderConnected", m.encoderConnected],
                    encoderAbsoluteAngle = table[prefix + "encoderAbsoluteAngle", m.encoderAbsoluteAngle]
                )
            }
            gyroInputs = GyroInput(
                gyroConnected =  table["Gyro/gyroConnected", gyroInputs.gyroConnected],
                yaw = table["Gyro/yaw", gyroInputs.yaw],
                yawRate = table["Gyro/yawRate", gyroInputs.yawRate],
                pitch = table["Gyro/pitch", gyroInputs.pitch],
                pitchRate = table["Gyro/pitchRate", gyroInputs.pitchRate],
                roll = table["Gyro/roll", gyroInputs.roll],
                rollRate = table["Gyro/rollRate", gyroInputs.rollRate],
                xAccel = table["Gyro/xAccel", gyroInputs.xAccel],
                yAccel = table["Gyro/yAccel", gyroInputs.yAccel]
            )
        }
    }

    class ModuleInput (
        var driveConnected: Boolean,
        var driveVelocity: AngularVelocity,
        var driveAccel: AngularAcceleration,
        var driveSupplyCurrentAmps: Double,
        var driveStatorCurrentAmps: Double,
        var driveAppliedVolts: Double,
        var driveTemp: Double,

        var steerConnected: Boolean,
        var steerVelocity: AngularVelocity,
        var steerAccel: AngularAcceleration,
        var steerSupplyCurrentAmps: Double,
        var steerStatorCurrentAmps: Double,
        var steerAppliedVolts: Double,
        var steerTemp: Double,

        var encoderConnected: Boolean,
        var encoderAbsoluteAngle: Angle
    ) {
        companion object{
            val Empty = ModuleInput(
                false,
                0.0.degreesPerSecond,
                0.0.degreesPerSecondPerSecond,
                0.0,
                0.0,
                0.0,
                0.0,
                false,
                0.0.degreesPerSecond,
                0.0.degreesPerSecondPerSecond,
                0.0,
                0.0,
                0.0,
                0.0,
                false,
                0.0.degrees
            )
        }
    }

    class GyroInput (
        var gyroConnected: Boolean,
        var yaw: Angle,
        var yawRate: AngularVelocity,
        var pitch: Angle,
        var pitchRate: AngularVelocity,
        var roll: Angle,
        var rollRate: AngularVelocity,
        var xAccel: LinearAcceleration,
        var yAccel: LinearAcceleration,
    ) {
        companion object {
            val Empty = GyroInput(
                false,
                0.0.degrees,
                0.0.degrees.perSecond,
                0.0.degrees,
                0.0.degrees.perSecond,
                0.0.degrees,
                0.0.degrees.perSecond,
                0.0.feet.perSecondPerSecond,
                0.0.feet.perSecondPerSecond
            )
        }
    }
}
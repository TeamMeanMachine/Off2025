package frc.team2471.off2025.util

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState

inline val ChassisSpeeds.translation: Translation2d get() = Translation2d(this.vxMetersPerSecond, this.vyMetersPerSecond)
fun SwerveDriveKinematics.toChassisSpeedsK(speeds: Array<SwerveModuleState>): ChassisSpeeds {
    return this.toChassisSpeeds(*speeds) //Intellij thinks this is an error. which is lame...
}

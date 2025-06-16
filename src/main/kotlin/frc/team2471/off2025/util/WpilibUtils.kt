package frc.team2471.off2025.util

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds

inline val ChassisSpeeds.translation: Translation2d get() = Translation2d(this.vxMetersPerSecond, this.vyMetersPerSecond)

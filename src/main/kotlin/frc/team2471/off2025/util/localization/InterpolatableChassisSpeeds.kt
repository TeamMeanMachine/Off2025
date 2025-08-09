package frc.team2471.off2025.util.localization

import edu.wpi.first.math.interpolation.Interpolatable
import edu.wpi.first.math.kinematics.ChassisSpeeds

class InterpolatableChassisSpeeds(
    vxMetersPerSecond: Double,
    vyMetersPerSecond: Double,
    omegaRadiansPerSecond: Double
): ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond),
    Interpolatable<InterpolatableChassisSpeeds> {

    override fun interpolate(endValue: InterpolatableChassisSpeeds, t: Double): InterpolatableChassisSpeeds {
        if (t < 0) {
            return this
        }
        if (t >= 1) {
            return endValue
        }
        return InterpolatableChassisSpeeds(
            (1 - t) * vxMetersPerSecond + t * endValue.vxMetersPerSecond,
            (1 - t) * vyMetersPerSecond + t * endValue.vyMetersPerSecond,
            (1 - t) * omegaRadiansPerSecond + t * endValue.omegaRadiansPerSecond
        )
    }

    companion object {
        fun fromChassisSpeeds(cs: ChassisSpeeds): InterpolatableChassisSpeeds {
            return InterpolatableChassisSpeeds(
                cs.vxMetersPerSecond, cs.vyMetersPerSecond, cs.omegaRadiansPerSecond
            )
        }
    }
}
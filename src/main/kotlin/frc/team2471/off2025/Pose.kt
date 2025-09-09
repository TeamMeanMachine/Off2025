package frc.team2471.off2025

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.inches
import frc.team2471.off2025.util.units.wrap
import kotlin.unaryMinus

data class Pose(var elevatorHeight: Distance, var armAngle: Angle, var pivotAngle: Angle) {
    companion object {
        val current: Pose
            get() = Pose(Armavator.currentHeight, Armavator.currentArmAngle, Armavator.pivotEncoderAngle)
        val DRIVE = Pose(0.0.inches, 0.0.degrees, -90.0.degrees)
        val SCORE_L1 = Pose(0.0.inches, 60.0.degrees, -150.0.degrees)
        val SCORE_L2 = Pose(0.0.inches, 20.2.degrees, 90.0.degrees)
        val SCORE_L3 = Pose(16.75.inches, 20.2.degrees, 90.0.degrees)
        val SCORE_L4 = Pose(58.0.inches, 57.0.degrees, 90.0.degrees)

        val ALGAE_DESCORE_LOW = Pose(16.0.inches, 72.0.degrees, -90.0.degrees)
        val ALGAE_DESCORE_HIGH = Pose(34.0.inches, 72.0.degrees, -90.0.degrees)
        val ALGAE_DESCORE_HIGH_FLIPPED = Pose(ALGAE_DESCORE_HIGH.elevatorHeight - 7.0.inches, ALGAE_DESCORE_HIGH.armAngle, -ALGAE_DESCORE_HIGH.pivotAngle)

        val INTAKE_CORAL_STATION = Pose(0.0.inches, 30.0.degrees, 180.0.degrees)
        val INTAKE_GROUND = Pose(0.0.inches, 113.0.degrees, 180.0.degrees)

        val DRIVE_PIVOT_ONE_THIRD_TEST = Pose(0.0.inches, 0.0.degrees, (120-90.0).degrees)
        val DRIVE_PIVOT_TWO_THIRDS_TEST = Pose(0.0.inches, 0.0.degrees, (240-90.0).degrees)
    }

    fun reflect() = Pose(elevatorHeight, -armAngle, (pivotAngle - 180.0.degrees).wrap())
}

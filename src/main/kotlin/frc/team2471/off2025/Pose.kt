package frc.team2471.off2025

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.inches
import frc.team2471.off2025.util.units.wrap

data class Pose(val elevatorHeight: Distance, val armAngle: Angle, val pivotAngle: Angle, ) {
    companion object {
        val current: Pose
            get() = Pose(Armavator.currentHeight, Armavator.currentArmAngle, 0.0.degrees)
        val DRIVE = Pose(0.0.inches, 0.0.degrees, -90.0.degrees)
        val SCORE_L1 = Pose(0.0.inches, 60.0.degrees, -150.0.degrees)
        val SCORE_L2 = Pose(0.0.inches, 20.2.degrees, 90.0.degrees)
        val SCORE_L3 = Pose(16.75.inches, 20.2.degrees, 90.0.degrees)
        val SCORE_L4 = Pose(58.0.inches, 57.0.degrees, 90.0.degrees)

        val INTAKE_GROUND = Pose(0.0.inches, 113.0.degrees, 180.0.degrees)

        val DRIVE_PIVOT_ONE_THIRD_TEST = Pose(0.0.inches, 0.0.degrees, (120-90.0).degrees)
        val DRIVE_PIVOT_TWO_THIRDS_TEST = Pose(0.0.inches, 0.0.degrees, (240-90.0).degrees)
    }

    fun reflect() = Pose(elevatorHeight, -armAngle, (pivotAngle - 180.0.degrees).wrap())
}

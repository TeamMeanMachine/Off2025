package frc.team2471.off2025.util.vision

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import frc.team2471.off2025.util.units.asDegrees
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.radians
import frc.team2471.off2025.util.units.wrap
import org.littletonrobotics.junction.Logger
import kotlin.math.IEEErem
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.sign

fun getCoralAngle(corners: DoubleArray, center: Translation2d): Angle {
    if (corners.isEmpty()) {
        return 0.0.degrees
    }

    val cornersX = corners.filterIndexed { index, _ -> index % 2 == 0 }
    val cornersY = corners.filterIndexed { index, _ -> index % 2 == 1 }

    val cornersTranslation = cornersX.mapIndexed { index, value -> Translation2d(value, cornersY[index]) }.sortedBy { it.x }


    val dists = DoubleArray(cornersTranslation.size - 1) {0.0}

    for (i in 1..(cornersTranslation.size - 1)) {
        dists[i - 1] = cornersTranslation[0].getDistance(cornersTranslation[i])
    }

    val distThreshold = dists.max() * 0.6

    val group1: MutableList<Translation2d> = mutableListOf(cornersTranslation[0])
    val group2: MutableList<Translation2d> = mutableListOf()

    for (i in 1..cornersTranslation.size - 1) {
        if (dists[i - 1] > distThreshold) {
            group2.add(cornersTranslation[i])
        } else {
            group1.add(cornersTranslation[i])
        }
    }

    Logger.recordOutput("Group 1", *group1.toTypedArray())
    Logger.recordOutput("Group 2", *group2.toTypedArray())


    val group1Center = Translation2d(group1.map {it.x}.average(), group1.map {it.y}.average())
    val group2Center = Translation2d(group2.map {it.x}.average(), group2.map {it.y}.average())

    return atan2(group1Center.y - group2Center.y, group1Center.x - group2Center.x).radians.asDegrees.let {
        (if (it.absoluteValue > 90.0) it - it.sign * 180.0 else it).degrees
    }
}
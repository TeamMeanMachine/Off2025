// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.team2471.off2025

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import frc.team2471.off2025.util.asMeters
import frc.team2471.off2025.util.degrees
import frc.team2471.off2025.util.inches
import frc.team2471.off2025.util.quix.Fiducials
import java.util.Map

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
object Constants {
    object Field {
        val fieldLength: Double = Units.inchesToMeters(12 * 57 + 6.875) // From sim
        val fieldWidth: Double = Units.inchesToMeters((12 * 26 + 5).toDouble()) // From sim

        // Reef measurements
        val robotReefWallPrescoreClearanceDistance: Double = Units.inchesToMeters(40.0)
        val robotReefWallPrescoreOffsetDistance: Double = Units.inchesToMeters(44.0)
        val robotReefWallScoringOffsetDistance: Double = Units.inchesToMeters(30.0)
        val robotReefWallL1ScoringOffsetDistance: Double = Units.inchesToMeters(28.0)
        val L1AngleOffset: Rotation2d = Rotation2d.fromDegrees(0.0)
        val robotAlgaeIntakeOffsetDistance: Double = Units.inchesToMeters(36.0)
        val robotReefWallL4ScoringOffsetDistance: Double = Units.inchesToMeters(26.0)
        val netScoringOffsetDistance: Double = Units.inchesToMeters(12.0) // TBD
        val processorScoringOffsetDistance: Double = Units.inchesToMeters(24.0) // TBD
        val tagToLeftReefTipTransform: Transform2d =
            Transform2d(Units.inchesToMeters(-2.0), Units.inchesToMeters(-6.5), Rotation2d.kZero)
        val tagToRightReefTipTransform: Transform2d =
            Transform2d(Units.inchesToMeters(-2.0), Units.inchesToMeters(6.5), Rotation2d.kZero)
        val reefCenterBlue: Pose2d = Pose2d(
            Fiducials.aprilTagFiducials[20]
                .pose
                .toPose2d()
                .getTranslation()
                .plus(Fiducials.aprilTagFiducials[17].pose.toPose2d().getTranslation())
                .div(2.0),
            Rotation2d.kZero
        )
        val reefCenterRed: Pose2d = Pose2d(
            Fiducials.aprilTagFiducials[9]
                .pose
                .toPose2d()
                .getTranslation()
                .plus(Fiducials.aprilTagFiducials[6].pose.toPose2d().getTranslation())
                .div(2.0),
            Rotation2d.kZero
        )

        // Tag shennaniganery
        private val blueToRedMap: MutableMap<Int?, Int?> = Map.ofEntries<Int?, Int?>(
            Map.entry<Int?, Int?>(20, 11),
            Map.entry<Int?, Int?>(19, 6),
            Map.entry<Int?, Int?>(18, 7),
            Map.entry<Int?, Int?>(17, 8),
            Map.entry<Int?, Int?>(22, 9),
            Map.entry<Int?, Int?>(21, 10)
        )

        fun getReefTagForAlliance(tagID: Int, isBlue: Boolean): Int {
            if (isBlue) {
                if (blueToRedMap.containsValue(tagID)) {
                    for (entry in blueToRedMap.entries) {
                        if (entry.value == tagID) {
                            return entry.key!!
                        }
                    }
                }
                return tagID
            } else {
                if (blueToRedMap.containsKey(tagID)) {
                    return blueToRedMap.get(tagID)!!
                }

                return tagID
            }
        }
    }


    // Need to be more accurate
    val frontRightCamPose: Transform3d = Transform3d(
        Translation3d(
            9.375.inches.asMeters,
            -4.125.inches.asMeters,
            7.745.inches.asMeters
        ), Rotation3d(
            0.0.degrees,
            (-20.0).degrees,
            -8.0.degrees
        )
    )

    val frontLeftCamPose: Transform3d = Transform3d(
        Translation3d(
            9.375.inches.asMeters,
            0.825.inches.asMeters,
            7.745.inches.asMeters
        ), Rotation3d(
            0.0.degrees,
            (-20.0).degrees,
            8.0.degrees
        )
    )

    val backLeftCamPose: Transform3d = Transform3d(
        Translation3d(
            -9.375.inches.asMeters,
            0.825.inches.asMeters,
            7.745.inches.asMeters
        ), Rotation3d(
            0.0.degrees,
            (-20.0).degrees,
            (180.0 - 8).degrees
        )
    )

    val backRightCamPose: Transform3d = Transform3d(
        Translation3d(
            -9.375.inches.asMeters,
            -4.0.inches.asMeters,
            7.745.inches.asMeters
        ), Rotation3d(
            0.0.degrees,
            (-20.0).degrees,
            (180.0 + 8).degrees
        )
    )
}

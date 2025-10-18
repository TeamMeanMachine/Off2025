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

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
object Constants {

    val limelightPose = Transform3d(
        Translation3d(
            0.254, -0.1335, 0.635
        ), Rotation3d(
            180.0.degrees, 35.0.degrees, 0.0.degrees
        )
    )
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

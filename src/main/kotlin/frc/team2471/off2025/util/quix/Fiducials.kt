package frc.team2471.off2025.util.quix

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import frc.team2471.off2025.util.vision.Fiducial

object Fiducials {
    // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
    private val aprilTagSize = Units.inchesToMeters(6.5) // m

    val aprilTagFiducials: Array<Fiducial> = arrayOf(
        Fiducial(
            Fiducial.Type.APRILTAG,
            1,
            Pose3d(
                Units.inchesToMeters(657.37),
                Units.inchesToMeters(25.80),
                Units.inchesToMeters(58.50),
                Rotation3d(0.0, 0.0, Math.toRadians(126.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            2,
            Pose3d(
                Units.inchesToMeters(657.37),
                Units.inchesToMeters(291.20),
                edu.wpi.first.math.util.Units.inchesToMeters(58.50),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(234.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            3,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(455.15),
                edu.wpi.first.math.util.Units.inchesToMeters(317.15),
                edu.wpi.first.math.util.Units.inchesToMeters(51.25),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(270.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            4,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(365.20),
                edu.wpi.first.math.util.Units.inchesToMeters(241.64),
                edu.wpi.first.math.util.Units.inchesToMeters(73.54),
                Rotation3d(0.0, java.lang.Math.toRadians(30.0), java.lang.Math.toRadians(0.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            5,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(365.20),
                edu.wpi.first.math.util.Units.inchesToMeters(75.39),
                edu.wpi.first.math.util.Units.inchesToMeters(73.54),
                Rotation3d(0.0, java.lang.Math.toRadians(30.0), java.lang.Math.toRadians(0.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            6,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(530.49),
                edu.wpi.first.math.util.Units.inchesToMeters(130.17),
                edu.wpi.first.math.util.Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(300.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            7,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(546.87),
                edu.wpi.first.math.util.Units.inchesToMeters(158.50),
                edu.wpi.first.math.util.Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(0.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            8,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(530.49),
                edu.wpi.first.math.util.Units.inchesToMeters(186.83),
                edu.wpi.first.math.util.Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(60.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            9,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(497.77),
                edu.wpi.first.math.util.Units.inchesToMeters(186.83),
                edu.wpi.first.math.util.Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(120.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            10,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(481.39),
                edu.wpi.first.math.util.Units.inchesToMeters(158.50),
                edu.wpi.first.math.util.Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(180.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            11,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(497.77),
                edu.wpi.first.math.util.Units.inchesToMeters(130.17),
                edu.wpi.first.math.util.Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(240.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            12,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(33.51),
                edu.wpi.first.math.util.Units.inchesToMeters(25.80),
                edu.wpi.first.math.util.Units.inchesToMeters(58.50),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(54.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            13,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(33.51),
                edu.wpi.first.math.util.Units.inchesToMeters(291.20),
                edu.wpi.first.math.util.Units.inchesToMeters(58.50),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(306.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            14,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(325.68),
                edu.wpi.first.math.util.Units.inchesToMeters(241.64),
                edu.wpi.first.math.util.Units.inchesToMeters(73.54),
                Rotation3d(0.0, java.lang.Math.toRadians(30.0), java.lang.Math.toRadians(180.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            15,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(325.68),
                edu.wpi.first.math.util.Units.inchesToMeters(75.39),
                edu.wpi.first.math.util.Units.inchesToMeters(73.54),
                Rotation3d(0.0, java.lang.Math.toRadians(30.0), java.lang.Math.toRadians(180.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            16,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(235.73),
                edu.wpi.first.math.util.Units.inchesToMeters(-0.15),
                edu.wpi.first.math.util.Units.inchesToMeters(51.25),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(90.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            17,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(160.39),
                edu.wpi.first.math.util.Units.inchesToMeters(130.17),
                edu.wpi.first.math.util.Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(240.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            18,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(144.00),
                edu.wpi.first.math.util.Units.inchesToMeters(158.50),
                edu.wpi.first.math.util.Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(180.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            19,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(160.39),
                edu.wpi.first.math.util.Units.inchesToMeters(186.83),
                edu.wpi.first.math.util.Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(120.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            20,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(193.10),
                edu.wpi.first.math.util.Units.inchesToMeters(186.83),
                edu.wpi.first.math.util.Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(60.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            21,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(209.49),
                edu.wpi.first.math.util.Units.inchesToMeters(158.50),
                edu.wpi.first.math.util.Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(0.0))
            ),
            Fiducials.aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            22,
            Pose3d(
                edu.wpi.first.math.util.Units.inchesToMeters(193.10),
                edu.wpi.first.math.util.Units.inchesToMeters(130.17),
                edu.wpi.first.math.util.Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, java.lang.Math.toRadians(300.0))
            ),
            Fiducials.aprilTagSize
        )
    )

    val blueCoralStationTags: kotlin.Array<Fiducial?> = kotlin.arrayOf<Fiducial?>(
        Fiducials.aprilTagFiducials[11], Fiducials.aprilTagFiducials[12]
    )
    val redCoralStationTags: kotlin.Array<Fiducial?> =
        kotlin.arrayOf<Fiducial?>(Fiducials.aprilTagFiducials[0], Fiducials.aprilTagFiducials[1])
}

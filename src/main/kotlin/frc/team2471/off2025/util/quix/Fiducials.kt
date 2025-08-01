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
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            2,
            Pose3d(
                Units.inchesToMeters(657.37),
                Units.inchesToMeters(291.20),
                Units.inchesToMeters(58.50),
                Rotation3d(0.0, 0.0, Math.toRadians(234.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            3,
            Pose3d(
                Units.inchesToMeters(455.15),
                Units.inchesToMeters(317.15),
                Units.inchesToMeters(51.25),
                Rotation3d(0.0, 0.0, Math.toRadians(270.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            4,
            Pose3d(
                Units.inchesToMeters(365.20),
                Units.inchesToMeters(241.64),
                Units.inchesToMeters(73.54),
                Rotation3d(0.0, Math.toRadians(30.0), Math.toRadians(0.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            5,
            Pose3d(
                Units.inchesToMeters(365.20),
                Units.inchesToMeters(75.39),
                Units.inchesToMeters(73.54),
                Rotation3d(0.0, Math.toRadians(30.0), Math.toRadians(0.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            6,
            Pose3d(
                Units.inchesToMeters(530.49),
                Units.inchesToMeters(130.17),
                Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, Math.toRadians(300.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            7,
            Pose3d(
                Units.inchesToMeters(546.87),
                Units.inchesToMeters(158.50),
                Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, Math.toRadians(0.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            8,
            Pose3d(
                Units.inchesToMeters(530.49),
                Units.inchesToMeters(186.83),
                Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, Math.toRadians(60.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            9,
            Pose3d(
                Units.inchesToMeters(497.77),
                Units.inchesToMeters(186.83),
                Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, Math.toRadians(120.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            10,
            Pose3d(
                Units.inchesToMeters(481.39),
                Units.inchesToMeters(158.50),
                Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, Math.toRadians(180.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            11,
            Pose3d(
                Units.inchesToMeters(497.77),
                Units.inchesToMeters(130.17),
                Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, Math.toRadians(240.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            12,
            Pose3d(
                Units.inchesToMeters(33.51),
                Units.inchesToMeters(25.80),
                Units.inchesToMeters(58.50),
                Rotation3d(0.0, 0.0, Math.toRadians(54.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            13,
            Pose3d(
                Units.inchesToMeters(33.51),
                Units.inchesToMeters(291.20),
                Units.inchesToMeters(58.50),
                Rotation3d(0.0, 0.0, Math.toRadians(306.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            14,
            Pose3d(
                Units.inchesToMeters(325.68),
                Units.inchesToMeters(241.64),
                Units.inchesToMeters(73.54),
                Rotation3d(0.0, Math.toRadians(30.0), Math.toRadians(180.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            15,
            Pose3d(
                Units.inchesToMeters(325.68),
                Units.inchesToMeters(75.39),
                Units.inchesToMeters(73.54),
                Rotation3d(0.0, Math.toRadians(30.0), Math.toRadians(180.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            16,
            Pose3d(
                Units.inchesToMeters(235.73),
                Units.inchesToMeters(-0.15),
                Units.inchesToMeters(51.25),
                Rotation3d(0.0, 0.0, Math.toRadians(90.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            17,
            Pose3d(
                Units.inchesToMeters(160.39),
                Units.inchesToMeters(130.17),
                Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, Math.toRadians(240.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            18,
            Pose3d(
                Units.inchesToMeters(144.00),
                Units.inchesToMeters(158.50),
                Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, Math.toRadians(180.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            19,
            Pose3d(
                Units.inchesToMeters(160.39),
                Units.inchesToMeters(186.83),
                Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, Math.toRadians(120.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            20,
            Pose3d(
                Units.inchesToMeters(193.10),
                Units.inchesToMeters(186.83),
                Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, Math.toRadians(60.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            21,
            Pose3d(
                Units.inchesToMeters(209.49),
                Units.inchesToMeters(158.50),
                Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, Math.toRadians(0.0))
            ),
            aprilTagSize
        ),
        Fiducial(
            Fiducial.Type.APRILTAG,
            22,
            Pose3d(
                Units.inchesToMeters(193.10),
                Units.inchesToMeters(130.17),
                Units.inchesToMeters(12.13),
                Rotation3d(0.0, 0.0, Math.toRadians(300.0))
            ),
            aprilTagSize
        )
    )

    val blueCoralStationTags: Array<Fiducial?> = arrayOf(aprilTagFiducials[11], aprilTagFiducials[12])
    val redCoralStationTags: Array<Fiducial?> = arrayOf(aprilTagFiducials[0], aprilTagFiducials[1])
}

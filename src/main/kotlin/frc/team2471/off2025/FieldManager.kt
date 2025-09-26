package frc.team2471.off2025

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Timer
import frc.team2471.off2025.util.isBlueAlliance
import frc.team2471.off2025.util.units.absoluteValue
import frc.team2471.off2025.util.units.asFeet
import frc.team2471.off2025.util.units.asMeters
import frc.team2471.off2025.util.units.asRotation2d
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.feet
import frc.team2471.off2025.util.units.inches
import frc.team2471.off2025.util.isRedAlliance
import frc.team2471.off2025.util.units.meters
import frc.team2471.off2025.util.math.mirrorYAxis
import frc.team2471.off2025.util.math.round
import frc.team2471.off2025.util.math.toPose2d
import frc.team2471.off2025.util.units.UTranslation2d
import frc.team2471.off2025.util.units.wrap
import org.littletonrobotics.junction.Logger
import kotlin.math.floor

object FieldManager {
    val aprilTagFieldLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
    val allAprilTags = aprilTagFieldLayout.tags

    val fieldWidth = aprilTagFieldLayout.fieldWidth.meters
    val fieldLength = aprilTagFieldLayout.fieldLength.meters

    val fieldDimensions = UTranslation2d(fieldLength, fieldWidth)

    val fieldHalfWidth = fieldWidth / 2.0
    val fieldHalfLength = fieldLength / 2.0

    val fieldCenter = fieldDimensions / 2.0

    //Blue Reef AprilTags
    val reefAprilTagsBlue = allAprilTags.filter { it.ID in 17..22 }
    val reefAprilTagPositionsBlue = reefAprilTagsBlue.map {it.pose.toPose2d()}

    //Red Reef AprilTags
    val reefAprilTagsRed = allAprilTags.filter { it.ID in 6..11 }
    val reefAprilTagPositionsRed = reefAprilTagsRed.map {it.pose.toPose2d()}

    val reefCenterBlue: Translation2d = (allAprilTags[20].pose.toPose2d().translation + allAprilTags[17].pose.toPose2d().translation) / 2.0
    val reefCenterRed: Translation2d = (allAprilTags[9].pose.toPose2d().translation + allAprilTags[6].pose.toPose2d().translation) / 2.0

    // Align Point Arrays
    private val alignPositionsRightRed = mutableListOf<Pose2d>()
    private val alignPositionsLeftRed = mutableListOf<Pose2d>()

    private val alignPositionsRightBlue = mutableListOf<Pose2d>()
    private val alignPositionsLeftBlue = mutableListOf<Pose2d>()

    private val alignPositionsRightL4Red = mutableListOf<Pose2d>()
    private val alignPositionsLeftL4Red = mutableListOf<Pose2d>()

    private val alignPositionsRightL4Blue = mutableListOf<Pose2d>()
    private val alignPositionsLeftL4Blue = mutableListOf<Pose2d>()

    private val alignPositionsL1Red = mutableListOf<Pose2d>()
    private val alignPositionsL1Blue = mutableListOf<Pose2d>()

    private val alignPositionsAlgaeRed = mutableListOf<Pair<Pose2d, AlgaeLevel>>()
    private val alignPositionsAlgaeBlue = mutableListOf<Pair<Pose2d, AlgaeLevel>>()

    // L3 and L2
    private val alignRightOffset = Transform2d(19.0.inches, 6.5.inches, Rotation2d()) //L2, L3
    private val alignLeftOffset = alignRightOffset.mirrorYAxis()

    // L4
    private val alignRightL4Offset = Transform2d(25.0.inches, 6.5.inches, Rotation2d()) //L4 ONLY
    private val alignLeftL4Offset = alignRightL4Offset.mirrorYAxis()

    // L1
    private val alignL1Offset = Transform2d(19.0.inches, 19.0.inches, Rotation2d())

    // Algae
    private val alignAlgaeOffset = Transform2d(28.0.inches, 6.5.inches, Rotation2d())

    // Barge
    private val blueBargeAlignX = 25.0.feet
    private val bargeAlignPointsBlue = Pair(Translation2d(blueBargeAlignX, 24.5.feet), Translation2d(blueBargeAlignX, 15.0.feet))
    private val bargeAlignPointsRed = Pair(bargeAlignPointsBlue.first.rotateAroundField(), bargeAlignPointsBlue.second.rotateAroundField())

    val bargeAlignPoints: Pair<Translation2d, Translation2d> get() = if (isRedAlliance) bargeAlignPointsRed else bargeAlignPointsBlue

    // Amp
    private val ampAlignPointBlue: Pose2d = Pose2d(19.6.feet, 0.75.meters + 12.0.inches, 90.0.degrees.asRotation2d)
    private val ampAlignPointRed: Pose2d = ampAlignPointBlue.rotateAround(fieldCenter, 180.0.degrees.asRotation2d)


    init {
        println("FieldManager init: I see ${allAprilTags.size} AprilTags and the field is ${fieldLength.asFeet.round(3)} feet long")
        val startTime = Timer.getFPGATimestamp()


        for (face in 0..5) {
            val redReefAprilTagID = 10 - face
            println("face $redReefAprilTagID")
            val tagPose = allAprilTags[redReefAprilTagID].pose.toPose2d()
            alignPositionsRightRed.add(face, tagPose.transformBy(alignRightOffset))
            alignPositionsLeftRed.add(face, tagPose.transformBy(alignLeftOffset))
            alignPositionsRightL4Red.add(face, tagPose.transformBy(alignRightL4Offset))
            alignPositionsLeftL4Red.add(face, tagPose.transformBy(alignLeftL4Offset))
            alignPositionsL1Red.add(face, tagPose.transformBy(alignL1Offset))

            alignPositionsRightBlue.add(face, alignPositionsRightRed[face].rotateAround(fieldCenter, 180.0.degrees.asRotation2d))
            alignPositionsLeftBlue.add(face, alignPositionsLeftRed[face].rotateAround(fieldCenter, 180.0.degrees.asRotation2d))
            alignPositionsRightL4Blue.add(face, alignPositionsRightL4Red[face].rotateAround(fieldCenter, 180.0.degrees.asRotation2d))
            alignPositionsLeftL4Blue.add(face, alignPositionsLeftL4Red[face].rotateAround(fieldCenter, 180.0.degrees.asRotation2d))
            alignPositionsL1Blue.add(face, alignPositionsL1Red[face].rotateAround(fieldCenter, 180.0.degrees.asRotation2d))

            val algaeLevel = if (face.mod(2) == 0) AlgaeLevel.HIGH else AlgaeLevel.LOW
            alignPositionsAlgaeRed.add(face, Pair(tagPose.transformBy(alignAlgaeOffset), algaeLevel))
            alignPositionsAlgaeBlue.add(face, Pair(alignPositionsAlgaeRed[face].first.rotateAround(fieldCenter, 180.0.degrees.asRotation2d), algaeLevel))
        }
        val allAlignPositions = mutableListOf<Pose2d>()

        alignPositionsRightRed.toCollection(allAlignPositions)
        alignPositionsLeftRed.toCollection(allAlignPositions)
        alignPositionsRightBlue.toCollection(allAlignPositions)
        alignPositionsLeftBlue.toCollection(allAlignPositions)
        alignPositionsRightL4Red.toCollection(allAlignPositions)
        alignPositionsLeftL4Red.toCollection(allAlignPositions)
        alignPositionsRightL4Blue.toCollection(allAlignPositions)
        alignPositionsLeftL4Blue.toCollection(allAlignPositions)
        alignPositionsL1Red.toCollection(allAlignPositions)
        alignPositionsL1Blue.toCollection(allAlignPositions)
        alignPositionsAlgaeRed.map { it.first }.toCollection(allAlignPositions)
        alignPositionsAlgaeBlue.map { it.first }.toCollection(allAlignPositions)
        allAlignPositions.add(ampAlignPointRed)
        allAlignPositions.add(ampAlignPointBlue)

        println("Created ${allAlignPositions.size} align positions in ${(Timer.getFPGATimestamp() - startTime).round(4)} seconds")

        Logger.recordOutput("FieldManager/alignPositionL4Left", *alignPositionsLeftL4Red.toTypedArray())
        Logger.recordOutput("FieldManager/alignPositionLeft", *alignPositionsLeftRed.toTypedArray())
        Logger.recordOutput("FieldManager/alignPositionsAlgae", *alignPositionsAlgaeRed.map {it.first}.toTypedArray())

        Logger.recordOutput("FieldManager/allAlignPosition", *allAlignPositions.toTypedArray())

        Logger.recordOutput("FieldManager/fieldCenter", fieldCenter.toPose2d())
        Logger.recordOutput("FieldManager/fieldDimensions", fieldDimensions.toPose2d())
        Logger.recordOutput("FieldManager/allApriltags", *allAprilTags.map { it.pose }.toTypedArray())

        Logger.recordOutput("FieldManager/blueBargeAlign", *arrayOf(bargeAlignPointsBlue.first, bargeAlignPointsBlue.second))
        Logger.recordOutput("FieldManager/redBargeAlign", *arrayOf(bargeAlignPointsRed.first, bargeAlignPointsRed.second))

    }


    fun getHumanStationAlignHeading(pose: Pose2d): Pair<Angle, Boolean> {
        val goalHeading = (if (isRedAlliance) 54.0 else 126.0).degrees * if (pose.y.meters < fieldCenter.y) -1.0 else 1.0
        return if ((pose.rotation.measure - goalHeading).wrap().absoluteValue() > 90.0.degrees) {
            Pair(goalHeading + 180.0.degrees, true)

        } else {
            Pair(goalHeading, false)
        }
    }


    fun closestAlignPoint(pose: Pose2d, level: Level, side: ScoringSide? = null): Pair<Pose2d, Boolean> {
        val isRed = isRedAlliance
        val alignPositions = when (level) {
            Level.L4 -> {
                when (side) {
                    ScoringSide.RIGHT -> if (isRed) alignPositionsRightL4Red else alignPositionsRightL4Blue
                    ScoringSide.LEFT -> if (isRed) alignPositionsLeftL4Red else alignPositionsLeftL4Blue
                    else -> if (isRed) mutableListOf(*alignPositionsRightL4Red.toTypedArray(), *alignPositionsLeftL4Red.toTypedArray()) else mutableListOf(*alignPositionsRightL4Blue.toTypedArray(), *alignPositionsLeftL4Blue.toTypedArray())
                }
            }
            Level.L2, Level.L3 -> {
                when (side) {
                    ScoringSide.RIGHT -> if (isRed) alignPositionsRightRed else alignPositionsRightBlue
                    ScoringSide.LEFT -> if (isRed) alignPositionsLeftRed else alignPositionsLeftBlue
                    else -> if (isRed) mutableListOf(*alignPositionsRightRed.toTypedArray(), *alignPositionsLeftRed.toTypedArray()) else mutableListOf(*alignPositionsRightBlue.toTypedArray(), *alignPositionsLeftBlue.toTypedArray())
                }
            }
            Level.L1 -> if (isRed) alignPositionsL1Red else alignPositionsL1Blue
        }

        //Calculate closest distance
        var poseAndDistance: Pair<Pose2d, Double> = Pair(Pose2d(), Double.MAX_VALUE)
        alignPositions.forEach {
            val distance = it.translation.getDistance(pose.translation)
            if (distance < poseAndDistance.second) {
                poseAndDistance = Pair(it, distance)
            }
        }
        // optimize rotation
        var closestPose = poseAndDistance.first
        var isFlipped = true
        if ((pose.rotation.measure - closestPose.rotation.measure).wrap().absoluteValue() > 90.0.degrees) {
            isFlipped = false
            closestPose = Pose2d(closestPose.translation, closestPose.rotation.rotateBy(180.0.degrees.asRotation2d))
        }

        return Pair(closestPose, isFlipped)
    }

    fun getApproachAngle(robotPose: Pose2d): Angle {
        val reefCenter = if (isBlueAlliance) reefCenterBlue else reefCenterRed
        val rawAngle = (reefCenter - robotPose.translation).angle
        return (60.0 * floor((rawAngle.degrees + 30.0)/60.0)).degrees
    }

    fun ampAlignPoint(robotPose: Pose2d): Pose2d {
        var unwrappedPose = if (robotPose.onRedSide()) ampAlignPointRed else ampAlignPointBlue
        if ((robotPose.rotation.measure - unwrappedPose.rotation.measure).wrap().absoluteValue() > 90.0.degrees) {
            unwrappedPose = Pose2d(unwrappedPose.translation, unwrappedPose.rotation.rotateBy(180.0.degrees.asRotation2d))
        }
        return unwrappedPose
    }

    fun getClosestReefAlgae(robotPose: Pose2d): Triple<Pose2d, AlgaeLevel, Boolean> {
        val algaeAlignPoses = if (isRedAlliance) alignPositionsAlgaeRed else alignPositionsAlgaeBlue
        algaeAlignPoses.sortBy { it.first.translation.getDistance(robotPose.translation) }
        var closestPose = algaeAlignPoses.first()

        var isFlipped = true
        if ((robotPose.rotation.measure - closestPose.first.rotation.measure).wrap().absoluteValue() > 90.0.degrees) {
            isFlipped = false
            closestPose = Pair(
                Pose2d(
                    closestPose.first.translation,
                    closestPose.first.rotation.rotateBy(180.0.degrees.asRotation2d)
                ), closestPose.second
            )
        }

        return Triple(closestPose.first, closestPose.second, isFlipped)
    }

    /**
     * Reflects [Translation2d] across the midline of the field. Useful for mirrored field layouts (2023, 2024).
     * Units must be meters
     * @param doReflect Supplier to perform reflection. Default: true
     * @see Translation2d.rotateAroundField
     */
    fun Translation2d.reflectAcrossField(doReflect: () -> Boolean = { true }): Translation2d {
        return if (doReflect()) Translation2d(fieldLength.asMeters - x, y) else this
    }

    /**
     * Reflects [Pose2d] across the midline of the field. Useful for mirrored field layouts (2023, 2024).
     * Units must be meters
     * @param doReflect Supplier to perform reflection. Default: true
     * @see Pose2d.rotateAroundField
     */
    fun Pose2d.reflectAcrossField(doReflect: () -> Boolean = { true }): Pose2d {
        return if (doReflect()) Pose2d(fieldLength.asMeters - x, y, (rotation - 180.0.degrees.asRotation2d).wrap()) else this
    }

    /**
     * Rotates the [Translation2d] 180 degrees around the center of the field. Useful for reflected field layouts (2022, 2025).
     * Units must be meters
     * @param doRotate Supplier to perform rotation. Default: true
     * @see Translation2d.reflectAcrossField
     */
    fun Translation2d.rotateAroundField(doRotate: () -> Boolean = { true }): Translation2d {
        return if (doRotate()) this.rotateAround(fieldCenter, 180.0.degrees.asRotation2d) else this
    }

    /**
     * Rotates the [Pose2d] 180 degrees around the center of the field. Useful for reflected field layouts (2022, 2025).
     * Units must be meters
     * @param doRotate Supplier to perform rotation. Default: true
     * @see Pose2d.reflectAcrossField
     */
    fun Pose2d.rotateAroundField(doRotate: () -> Boolean = { true }): Pose2d {
        return if (doRotate()) this.rotateAround(fieldCenter, 180.0.degrees.asRotation2d) else this
    }

    /**
     * Returns if the [Translation2d] is on the red alliance side of the field.
     */
    fun Translation2d.onRedSide(): Boolean = this.x > fieldCenter.x.asMeters
    /**
     * Returns if the [Translation2d] is on the blue alliance side of the field.
     */
    fun Translation2d.onBlueSide(): Boolean = !this.onRedSide()
    /**
     * Returns if the [Translation2d] is closer to your current alliance's side of the field.
     */
    fun Translation2d.onFriendlyAllianceSide() = this.onRedSide() == isRedAlliance
    /**
     * Returns if the [Translation2d] is closer to your opponent alliance's side of the field.
     */
    fun Translation2d.onOpposingAllianceSide() = !this.onFriendlyAllianceSide()

    /**
     * Returns if the [Pose2d] is on the red alliance side of the field.
     */
    fun Pose2d.onRedSide(): Boolean = this.translation.onRedSide()
    /**
     * Returns if the [Pose2d] is on the blue alliance side of the field.
     */
    fun Pose2d.onBlueSide(): Boolean = !this.onRedSide()
    /**
     * Returns if the [Pose2d] is closer to your current alliance's side of the field.
     */
    fun Pose2d.onFriendlyAllianceSide() = this.translation.onFriendlyAllianceSide()
    /**
     * Returns if the [Pose2d] is closer to your opponent alliance's side of the field.
     */
    fun Pose2d.onOpposingAllianceSide() = !this.onFriendlyAllianceSide()



    enum class Level {
        L1,
        L2,
        L3,
        L4,
    }
    enum class ScoringSide {
        LEFT,
        RIGHT
    }
    enum class AlgaeLevel {
        LOW,
        HIGH
    }
}

package frc.team2471.off2025

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import frc.team2471.off2025.util.ApplyModuleStates
import frc.team2471.off2025.util.LoopLogger
import frc.team2471.off2025.util.asRotation2d
import frc.team2471.off2025.util.cube
import frc.team2471.off2025.util.degrees
import frc.team2471.off2025.util.inches
import frc.team2471.off2025.util.isReal
import frc.team2471.off2025.util.isRedAlliance
import frc.team2471.off2025.util.isSim
import frc.team2471.off2025.util.localization.PoseLocalizer
import frc.team2471.off2025.util.square
import frc.team2471.off2025.util.swerve.SwerveDriveSubsystem
import frc.team2471.off2025.util.vision.Fiducials
import frc.team2471.off2025.util.vision.PhotonVisionCamera
import frc.team2471.off2025.util.vision.PipelineConfig
import frc.team2471.off2025.util.vision.QuixVisionCamera
import frc.team2471.off2025.util.vision.QuixVisionSim
import gg.questnav.questnav.PoseFrame
import gg.questnav.questnav.QuestNav
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.Logger
import kotlin.jvm.optionals.getOrNull
import kotlin.math.hypot

object Drive: SwerveDriveSubsystem(TunerConstants.drivetrainConstants, *TunerConstants.moduleConfigs) {

    override var pose: Pose2d
        get() = savedState.Pose
        set(value) {
            resetPose(value)
            localizer.resetPose(value)
//            quest.setPose(value.transformBy(robotToQuestTransformMeters))
        }

    override var heading: Rotation2d
        get() = pose.rotation
        set(value) {
            println("resting heading to ${value.degrees}")
            resetRotation(value)
            // localizer reads delta rotation, it doesn't need to be called here
            quest.setPose(Pose2d(questPose.translation, value))
        }

    // Vision
    val cameras: List<QuixVisionCamera> = listOf(
        PhotonVisionCamera("FrontLeft", Constants.frontLeftCamPose, arrayOf(PipelineConfig())),
        PhotonVisionCamera("FrontRight", Constants.frontRightCamPose, arrayOf(PipelineConfig())),
        PhotonVisionCamera("BackLeft", Constants.backLeftCamPose, arrayOf(PipelineConfig())),
        PhotonVisionCamera("BackRight", Constants.backRightCamPose, arrayOf(PipelineConfig())),
    )

    val quest = QuestNav()
    var questSimConnected = true
    val robotToQuestTransformMeters = Transform2d(0.0.inches, 0.0.inches, Rotation2d()) // Rotation 2d should be 0
    var latestQuestResult: PoseFrame = PoseFrame(pose.transformBy(robotToQuestTransformMeters), 0.0, 0.0, 0)
        get() {
            val newData = quest.allUnreadPoseFrames
            if (newData.isNotEmpty()) {
                newData.sortByDescending { it.dataTimestamp }
                field = newData.first() ?: field
            }
            return field
        }
        private set

    val questPose: Pose2d
        get() = latestQuestResult.questPose.transformBy(robotToQuestTransformMeters.inverse())

    val localizer = PoseLocalizer(
//        Pose2d(),
        Fiducials.aprilTagFiducials,
        cameras
    )

    // Drive Feedback controllers
    override val autoPilot = createAPObject(Double.POSITIVE_INFINITY, 20.0, 0.5, 0.5.inches, 1.0.degrees)

    override val pathXController = PIDController(7.0, 0.0, 0.0)
    override val pathYController = PIDController(7.0, 0.0, 0.0)
    override val pathThetaController = PIDController(7.0, 0.0, 0.0)

    override val autoDriveToPointController = PIDController(3.0, 0.0, 0.1)
    override val teleopDriveToPointController = PIDController(3.0, 0.0, 0.1)

    override val driveAtAnglePIDController = PhoenixPIDController(7.7, 0.0, 0.072)

    /**
     * Returns [ChassisSpeeds] with a percentage power from the driver controller.
     * Performs [OI.unsnapAndDesaturateJoystick] to undo axis snapping and does squaring/cubing on the vectors.
     */
    override fun getJoystickPercentageSpeeds(): ChassisSpeeds {
        //make joystick pure circle
        val (cx, cy) = OI.unsnapAndDesaturateJoystick(OI.driveTranslationX, OI.driveTranslationY)

        //square drive input
        val power = hypot(cx, cy).square()
        val (x, y) = Pair(cx * power, cy * power)

        //cube rotation input
        val omega = OI.driveRotation.cube()

        return ChassisSpeeds(x, y, omega)
    }

    init {
        println("inside Drive init")

        // MUST start inside the field on bootup for accurate measurements due to a vision localizer bug.
        pose = Pose2d(3.0, 3.0, heading)

        zeroGyro()

        localizer.trackAllTags()

        finalInitialization()
    }

    override fun periodic() {
        LoopLogger.record("b4 Drive piodc")
        super.periodic() // Must call this
        LoopLogger.record("super Drive piodc")

        // Disabled actions
        if (Robot.isDisabled) {
            setControl(ApplyModuleStates()) //set module setpoints to their current position
        }

        // Update Vision
        cameras.forEach {
            it.updateInputs()
        }
        LoopLogger.record("Drive camera updateInputs")
        val questResult = latestQuestResult
        val questEstimate = if (isReal) {
            PoseLocalizer.QuestNavMeasurement(questResult.questPose.transformBy(robotToQuestTransformMeters.inverse()), questResult.dataTimestamp)
        } else {
            val timestamp = Utils.fpgaToCurrentTime(Timer.getTimestamp()) - 0.04
            val simQuestPose = samplePoseAt(timestamp).getOrNull()
            if (simQuestPose == null || !questSimConnected) null else PoseLocalizer.QuestNavMeasurement(simQuestPose, timestamp)
        }
        LoopLogger.record("Drive get questEstimate")
        localizer.updateWithLatestPoseEstimate(if (quest.isTracking || isSim) questEstimate else null)
        LoopLogger.record("Drive updateWithLatestPose")
        localizer.update(pose, cameras.map { it.latestMeasurement }, speeds)
        LoopLogger.record("Drive localizer")

        Logger.recordOutput("Swerve/Odometry", localizer.rawOdometryPose)
        Logger.recordOutput("Swerve/Quest", localizer.rawQuestPose ?: Pose2d())
        Logger.recordOutput("Swerve/FusedPose", localizer.fusedOdometryPose)
        Logger.recordOutput("Swerve/Localizer Raw", localizer.rawPose)
        Logger.recordOutput("Swerve/Localizer", localizer.pose)
        Logger.recordOutput("Swerve/SingleTagPose", localizer.singleTagPose)

        LoopLogger.record("Drive pirdc")
    }

    fun zeroGyro() {
        val wantedAngle = (if (isRedAlliance) 180.0.degrees else 0.0.degrees).asRotation2d
        println("zero gyro isRedAlliance  $isRedAlliance zeroing to ${wantedAngle.degrees} degrees")
        heading = wantedAngle
        println("heading: $heading")
    }

    @OptIn(DelicateCoroutinesApi::class)
    override fun simulationPeriodic() {
        LoopLogger.record("b4 Drive Sim piodic")
        GlobalScope.launch {
            updateSimState(0.02, 12.0)
            QuixVisionSim.updatePose(pose)
        }
        LoopLogger.record("Drive Sim piodic")
    }
}
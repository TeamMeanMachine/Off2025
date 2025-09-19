package frc.team2471.off2025

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat.N1
import edu.wpi.first.math.Nat.N3
import edu.wpi.first.math.Nat.N8
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import frc.team2471.off2025.util.ctre.ApplyModuleStates
import frc.team2471.off2025.util.control.LoopLogger
import frc.team2471.off2025.util.units.asRotation2d
import frc.team2471.off2025.util.math.cube
import frc.team2471.off2025.util.isBlueAlliance
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.inches
import frc.team2471.off2025.util.isReal
import frc.team2471.off2025.util.isRedAlliance
import frc.team2471.off2025.util.localization.PoseLocalizer
import frc.team2471.off2025.util.math.square
import frc.team2471.off2025.util.swerve.SwerveDriveSubsystem
import frc.team2471.off2025.util.vision.Fiducials
import frc.team2471.off2025.util.vision.limelight.LimelightCamera
import frc.team2471.off2025.util.vision.photonVision.PhotonVisionCamera
import frc.team2471.off2025.util.vision.PipelineConfig
import frc.team2471.off2025.util.vision.QuixVisionCamera
import frc.team2471.off2025.util.vision.QuixVisionSim
import gg.questnav.questnav.PoseFrame
import gg.questnav.questnav.QuestNav
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.Logger
import java.util.Optional
import kotlin.jvm.optionals.getOrNull

object Drive: SwerveDriveSubsystem(TunerConstants.drivetrainConstants, *TunerConstants.moduleConfigs) {

    // To reset position use this, also add other pose sources that need reset here.
    override var pose: Pose2d
        get() = savedState.Pose
        set(value) {
            tempQuestPose = value.transformBy(robotToQuestTransformMeters)
            resetQuestTranslation = true
            resetPose(value)
            localizer.resetPose(value) // Possibly not needed, but good for a quick response.
        }

    var tempQuestPose = Pose2d()
    var resetQuestTranslation = false

    override var heading: Rotation2d
        get() = pose.rotation
        set(value) {
            println("resting heading to ${value.degrees}")
            resetRotation(value)
            localizer.resetRotation(value) // Not needed and redundant but may prevent some heading bugs
            if (resetQuestTranslation) {
                quest.setPose(Pose2d(tempQuestPose.translation, value + robotToQuestTransformMeters.rotation))
                resetQuestTranslation = false
            } else {
                quest.setPose(questPose.transformBy(robotToQuestTransformMeters))
            }
        }

    // Vision
    val cameras: List<QuixVisionCamera> = listOf(
        LimelightCamera(
            "limelight-test",
            Constants.limelightPose,
            Optional.of(Matrix(N3(), N3(), doubleArrayOf(737.3071162812872,0.0,658.6108346810324,0.0,738.2142014335819,411.36513253891655,0.0,0.0,1.0))),
            Optional.of(Matrix(N8(), N1(), doubleArrayOf(0.11977275268536702,-0.13935436337469195,-0.000907565402687582,0.0005097193890789445,-0.052047409417428844,0.0,0.0,0.0)))
            ),
        PhotonVisionCamera("FrontLeft", Constants.frontLeftCamPose, arrayOf(PipelineConfig())),
        PhotonVisionCamera("FrontRight", Constants.frontRightCamPose, arrayOf(PipelineConfig())),
        PhotonVisionCamera("BackLeft", Constants.backLeftCamPose, arrayOf(PipelineConfig())),
        PhotonVisionCamera("BackRight", Constants.backRightCamPose, arrayOf(PipelineConfig())),
    )

    val quest = QuestNav()
    var simulateQuest = true
    val questConnected: Boolean
        get() = if (isReal) quest.isTracking else simulateQuest
    val robotToQuestTransformMeters = Transform2d(-12.0.inches, 12.0.inches, 180.0.degrees.asRotation2d) // Rotation 2d should be 0
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

    // Class that handles the merging of multiple vision sources and the odometry.
    val localizer = PoseLocalizer(
//        Pose2d(),
        Fiducials.aprilTagFiducials,
        cameras
    )

    private val translationRateTimer = Timer()
    private var prevTranslation = Translation2d()

    // Drive Feedback controllers
    override val autoPilot = createAPObject(Double.POSITIVE_INFINITY, 100.0, 2.0, 0.5.inches, 1.0.degrees)

    override val pathXController = PIDController(7.0, 0.0, 0.0)
    override val pathYController = PIDController(7.0, 0.0, 0.0)
    override val pathThetaController = PIDController(7.0, 0.0, 0.0)

    override val autoDriveToPointController = PIDController(3.0, 0.0, 0.1)
    override val teleopDriveToPointController = PIDController(3.0, 0.0, 0.1)

    override val driveAtAnglePIDController = PhoenixPIDController(7.7, 0.0, 0.072)

    /**
     * Returns [ChassisSpeeds] with a percentage power from the driver controller.
     */
    override fun getJoystickPercentageSpeeds(): ChassisSpeeds {
        val rawJoystick = OI.rawDriveTranslation
        // Square drive input and apply demoSpeed
        val power = rawJoystick.norm.square() * demoSpeed
        // Apply modified power to joystick vector and flip depending on alliance
        val joystickTranslation = rawJoystick * power * if (isBlueAlliance) -1.0 else 1.0

        val rawJoystickRotation = OI.driveRotation
        // Cube rotation input and apply demoSpeed
        val omega = rawJoystickRotation.cube() * demoSpeed

        return ChassisSpeeds(joystickTranslation.x, joystickTranslation.y, omega)
    }

    init {
        println("inside Drive init")

        // MUST start inside the field on bootup for accurate heading measurements due to a vision localizer bug.
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
            if (isReal) {
                modules.forEach {
                    it.steerMotor.setPosition(it.encoder.position.value)
                }
            }
        }

        LoopLogger.record("Drive after steer")

        // Update Vision
        cameras.forEach {
            it.updateInputs()
        }
        LoopLogger.record("Drive camera updateInputs")
        // Get the latest Quest result
        val questResult = latestQuestResult
        val questEstimate = if (isReal) {
            PoseLocalizer.QuestNavMeasurement(questResult.questPose.transformBy(robotToQuestTransformMeters.inverse()), questResult.dataTimestamp)
        } else {
            // Simulate a Quest result
            val trueTimestamp = Timer.getTimestamp() - 0.04
            val timestamp = Utils.fpgaToCurrentTime(trueTimestamp)
            val simQuestPose = samplePoseAt(timestamp).getOrNull()
            if (simQuestPose == null || !simulateQuest) null else PoseLocalizer.QuestNavMeasurement(simQuestPose, trueTimestamp)
        }
        LoopLogger.record("Drive get questEstimate")
        // Update poses from the Particle Filter pose estimate and quest measurements
        localizer.updateWithLatestPoseEstimate(if (questConnected) questEstimate else null)
        LoopLogger.record("Drive updateWithLatestPose")
        // Publish the latest camera data to NT so Particle Filter can use, also update poses from swerve odometry measurements.
//        val modifiedPose = Pose2d(pose.translation - (pose.translation - Translation2d(3.0, 3.0)) / 2.0 , pose.rotation)
        localizer.update(pose, cameras.map { it.latestMeasurement }, speeds)
        LoopLogger.record("Drive localizer")

        quest.commandPeriodic()

        Logger.recordOutput("Drive/QuestConnected", questConnected)

        // Log all the poses for debugging
        Logger.recordOutput("Swerve/Odometry", localizer.rawOdometryPose)
        Logger.recordOutput("Swerve/Quest", localizer.rawQuestPose ?: Pose2d())
        Logger.recordOutput("Swerve/FusedPose", localizer.fusedOdometryPose)
        Logger.recordOutput("Swerve/Localizer Raw", localizer.rawPose)
        Logger.recordOutput("Swerve/Localizer", localizer.pose)
        Logger.recordOutput("Swerve/SingleTagPose", localizer.singleTagPose)
        Logger.recordOutput("Drive/TestOdometry", pose)

        LoopLogger.record("Drive pirdc")
    }

    fun zeroGyro() {
        val wantedAngle = (if (isRedAlliance) 180.0.degrees else 0.0.degrees).asRotation2d
        println("zero gyro isRedAlliance  $isRedAlliance zeroing to ${wantedAngle.degrees} degrees")
        heading = wantedAngle
        println("heading: $heading")
    }

    fun resetOdometryToAbsolute() {
        println("resetting odometry to localizer pose")
        pose = localizer.pose
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
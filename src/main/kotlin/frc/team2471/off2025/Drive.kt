package frc.team2471.off2025

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.team2471.off2025.util.ApplyModuleStates
import frc.team2471.off2025.util.LoopLogger
import frc.team2471.off2025.util.asRotation2d
import frc.team2471.off2025.util.cube
import frc.team2471.off2025.util.degrees
import frc.team2471.off2025.util.inches
import frc.team2471.off2025.util.isRedAlliance
import frc.team2471.off2025.util.square
import frc.team2471.off2025.util.swerve.SwerveDriveSubsystem
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import kotlin.math.hypot

object Drive: SwerveDriveSubsystem(TunerConstants.drivetrainConstants, *TunerConstants.moduleConfigs) {

    override var pose: Pose2d
        get() = savedState.Pose
        set(value) {
            resetPose(value)
        }

    override var heading: Rotation2d
        get() = pose.rotation
        set(value) {
            println("resting heading to $value")
            resetRotation(value)
        }

    // Drive Feedback controllers
    override val autoPilot = createAPObject(Double.POSITIVE_INFINITY, 20.0, 0.5, 0.5.inches, 1.0.degrees)

    override val pathXController = PIDController(7.0, 0.0, 0.0)
    override val pathYController = PIDController(7.0, 0.0, 0.0)
    override val pathThetaController = PIDController(7.0, 0.0, 0.0)

    override val autoDriveToPointController = PIDController(3.0, 0.0, 0.1)
    override val teleopDriveToPointController = PIDController(3.0, 0.0, 0.1)

    override val driveAtAnglePIDController = PhoenixPIDController(7.4, 0.0, 0.0)

    /**
     * Returns [edu.wpi.first.math.kinematics.ChassisSpeeds] with a percentage power from the driver controller.
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
        zeroGyro()

        finalInitialization()
    }

    override fun periodic() {
        // Must call this v
        super.periodic()

        // Disabled actions
        if (Robot.isDisabled) {
            setControl(ApplyModuleStates()) //set module setpoints to their current position
        }

        LoopLogger.record("Drive periodic")
    }

    fun zeroGyro() {
        val wantedAngle = (if (isRedAlliance) 180.0.degrees else 0.0.degrees).asRotation2d
        println("zero gyro isRedAlliance  $isRedAlliance zeroing to ${wantedAngle.degrees} degrees")
        heading = wantedAngle
        println("yaw: $gyroYaw")
    }

    @OptIn(DelicateCoroutinesApi::class)
    override fun simulationPeriodic() {
        GlobalScope.launch {
            updateSimState(0.02, 12.0)
        }
    }
}
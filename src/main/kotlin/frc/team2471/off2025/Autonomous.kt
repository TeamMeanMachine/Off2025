package frc.team2471.off2025

import choreo.Choreo
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.off2025.FieldManager.Level
import frc.team2471.off2025.FieldManager.rotateAroundField
import frc.team2471.off2025.tests.elevatorJoystick
import frc.team2471.off2025.tests.elevatorSetpointTest
import frc.team2471.off2025.tests.joystickTest
import frc.team2471.off2025.tests.leftRightStaticFFTest
import frc.team2471.off2025.tests.sysIDPivot
import frc.team2471.off2025.tests.slipCurrentTest
import frc.team2471.off2025.tests.velocityVoltTest
import frc.team2471.off2025.util.control.commands.beforeWait
import frc.team2471.off2025.util.control.commands.deadlineCommand
import frc.team2471.off2025.util.control.commands.deferCommand
import frc.team2471.off2025.util.control.commands.runOnce
import frc.team2471.off2025.util.units.asSeconds
import frc.team2471.off2025.util.isRedAlliance
import frc.team2471.off2025.util.math.round
import frc.team2471.off2025.util.control.commands.sequenceCommand
import frc.team2471.off2025.util.control.commands.toCommand
import frc.team2471.off2025.util.control.commands.waitUntilCommand
import frc.team2471.off2025.util.units.asRotation2d
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.meters
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import kotlin.collections.forEach
import kotlin.io.path.listDirectoryEntries
import kotlin.io.path.name
import kotlin.jvm.optionals.getOrNull

object Autonomous {
    val paths: MutableMap<String, Trajectory<SwerveSample>> = findChoreoPaths()

    // Chooser for selecting autonomous commands
    private val autoChooser: LoggedDashboardChooser<AutoCommand?> = LoggedDashboardChooser<AutoCommand?>("Auto Chooser").apply {
        addOption("8 Foot Straight", AutoCommand(::eightFootStraight))
        addOption("6x6 Square", AutoCommand(::squarePathTest))
        addOption("3 L4 Right", AutoCommand(::threeL4Right) { Pose2d(7.19158.meters, 3.0.meters, 180.0.degrees.asRotation2d).rotateAroundField { isRedAlliance } })
    }
    // Chooser for test commands
    private val testChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("Test Chooser").apply {
        // Set up SysId routines and test command options
        addOption("Drive Translation SysId ALL", Drive.sysIDTranslationAll())
        addOption("Drive Rotation SysId ALL", Drive.sysIDRotationAll())
        addOption("Drive Steer SysId ALL", Drive.sysIDSteerAll())
        addOption("Set Angle Offsets", Drive.setAngleOffsets())
        addOption("JoystickTest", joystickTest())
        addOption("ElevatorSetpointTest", Armavator.elevatorSetpointTest())
        addOption("ElevatorJoystickTest", Armavator.elevatorJoystick())
        addOption("Drive Slip Current Test", Drive.slipCurrentTest())
        addOption("Armavator Pivot SysId ALL", Armavator.sysIDPivot())
        addOption("Drive L/R Static FF Test", Drive.leftRightStaticFFTest())
        addOption("Drive Velocity Volt Test", Drive.velocityVoltTest())
    }

    val selectedAuto: AutoCommand? get() = autoChooser.get()
    val autonomousCommand: Command? get() = if (!Drive.demoMode) selectedAuto?.commandSupplier?.invoke() else ({ println("DEMO MODE: Not running auto, no killing kids today.") }).toCommand()
    val testCommand: Command? get() = testChooser.get()

    /**
     * Initial value determines which side all choreo paths are made for.
     * False = all choreo paths are made on the blue side.
     * True = all choreo paths are made on the red side.
     */
    var isPathsRed = false
        private set

    private var prevPathRed: Boolean? = null
    private var prevAuto: AutoCommand? = null

    init {
        val startTime = RobotController.getMeasureFPGATime()
        val pathNameAndStartPose = mutableListOf<Pair<String, Pose2d>>()
        paths.forEach {
            pathNameAndStartPose.add(Pair(
                it.value.name(),
                it.value.sampleAt(0.001, false).get().pose
            ))
        }
        println("paths: $pathNameAndStartPose")
        println("reading ${paths.size} paths took ${(RobotController.getMeasureFPGATime() - startTime).asSeconds.round(6)} seconds.")
    }

    fun setDrivePositionToAutoStartPoseIfAutoChange() {
        if (prevAuto != selectedAuto) {
            setDrivePositionToAutoStartPose()
            prevAuto = selectedAuto
        }
    }

    fun setDrivePositionToAutoStartPose() {
        val startingPose = selectedAuto?.startingPoseSupplier?.invoke()
        if (startingPose != null) {
            println("resetting drive pose to auto start pose")
            Drive.pose = startingPose
        }
    }

    /**
     * Checks if the alliance color has changed and flips the paths if so
     */
    fun flipPathsIfAllianceChange() {
        if (prevPathRed != null) {
            if (prevPathRed != isRedAlliance) {
                flipPaths()
            }
        } else if (isRedAlliance != isPathsRed) { // Code always goes here for the first time because prevPathRed starts null
            flipPaths()
        }
    }

    /**
     * Flip the path so it is correct for the alliance color
     */
    private fun flipPaths() {
        println("flipping paths")
        paths.replaceAll { _, t -> t.flipped() }
        isPathsRed = !isPathsRed
        prevPathRed = isPathsRed
        setDrivePositionToAutoStartPose()
        println(paths.map { it.value.sampleAt(0.1, false)?.get()?.pose})
    }

    /**
     * Find all the paths in the choreo directory and return a list of them
     */
    private fun findChoreoPaths(): MutableMap<String, Trajectory<SwerveSample>> {
        return try {
            val map: MutableMap<String, Trajectory<SwerveSample>> = mutableMapOf()
            Filesystem.getDeployDirectory().toPath().resolve("choreo").listDirectoryEntries("*.traj").forEach {
                try {
                    val name = it.name.removeSuffix(".traj")
                    val traj = Choreo.loadTrajectory(name).getOrNull()
                    if (traj != null) {
                        @Suppress("UNCHECKED_CAST")
                        map[name] = traj as Trajectory<SwerveSample>
                    }
                } catch (e: Exception) {
                    println("failed to load path at $it")
                    println(e)
                }
            }
            println("loaded ${map.size} paths")
            map
        } catch (e: Exception) {
            println("failed to load any auto paths $e"); mutableMapOf()
        }
    }

    private fun eightFootStraight (): Command {
        return Drive.driveAlongChoreoPath(paths["8 foot"]!!, resetOdometry = true)
    }
    private fun squarePathTest (): Command {
        return Drive.driveAlongChoreoPath(paths["square"]!!, resetOdometry = true)
    }
/*    private fun threeL4Right(): Command {
        val path = paths["3 L4 Right"]!!
        return sequenceCommand(
            Drive.driveAlongChoreoPath(path.getSplit(0).get(), poseSupplier = {Drive.localizer.singleTagPose}, resetOdometry = true),
            { println("L4 Score 1") }.toCommand(),
            Drive.driveAlongChoreoPath(path.getSplit(1).get(), poseSupplier = {Drive.localizer.pose}),
            { println("Intake") }.toCommand(),
            Drive.driveAlongChoreoPath(path.getSplit(2).get(), poseSupplier = {Drive.localizer.singleTagPose}),
            { println("L4 Score 2") }.toCommand(),
            Drive.driveAlongChoreoPath(path.getSplit(3).get(), poseSupplier = {Drive.localizer.pose}),
            { println("Intake") }.toCommand(),
            Drive.driveAlongChoreoPath(path.getSplit(4).get(), poseSupplier = {Drive.localizer.singleTagPose}),
            { println("L4 Score 3") }.toCommand(),
        )
    }*/


    private fun threeL4Right(): Command {
        return deferCommand(Drive, Armavator) {
            println("inside L4 right ${Robot.timeSinceEnabled}")
            Drive.pose = Pose2d(7.191587924957275.meters, 3.0.meters, 180.0.degrees.asRotation2d).rotateAroundField { isRedAlliance }
            val path = paths["3 L4 Right"]!!
            val coralStationPose = path.getSplit(3).get().getFinalPose(false).get()
            sequenceCommand(
                autoScoreCoral(if (isRedAlliance) FieldManager.alignPositionsLeftL4Red[2] else FieldManager.alignPositionsLeftL4Blue[2], Level.L4),
                scoreAuto(),
                deadlineCommand(
                    waitUntilCommand(2.0) { Intake.hasCargo }.beforeWait(1.0),
                    Drive.driveToAutopilotPoint(coralStationPose, { Drive.localizer.odometryPose }, autopilotSupplier = Drive.fastAutoPilot),
                    runOnce {
                        Drive.resetOdometryToAbsolute()
                        val isFlipped = FieldManager.getHumanStationAlignHeading(Drive.localizer.pose).second
                        Intake.hasCargo = false
                        Intake.intakeState = IntakeState.INTAKING
                        Armavator.goToPose(Pose.INTAKE_CORAL_STATION, isFlipped, false)
                    }
                ),
                autoScoreCoral(Level.L4, FieldManager.ScoringSide.RIGHT),
                scoreAuto(),
                deadlineCommand(
                    waitUntilCommand(2.0) { Intake.hasCargo }.beforeWait(1.0),
                    Drive.driveToAutopilotPoint(coralStationPose, { Drive.localizer.odometryPose }, autopilotSupplier = Drive.fastAutoPilot),
                    runOnce {
                        Drive.resetOdometryToAbsolute()
                        val isFlipped = FieldManager.getHumanStationAlignHeading(Drive.localizer.pose).second
                        Intake.hasCargo = false
                        Intake.intakeState = IntakeState.INTAKING
                        Armavator.goToPose(Pose.INTAKE_CORAL_STATION, isFlipped, false)
                    }
                ),
                autoScoreCoral(Level.L4, FieldManager.ScoringSide.LEFT),
                scoreAuto(),
                runOnce {
                    Armavator.goToPose(Pose(Pose.SCORE_L4.elevatorHeight, 0.0.degrees, Pose.SCORE_L4.pivotAngle))
                }
            )
        }
    }

    class AutoCommand(val commandSupplier: () -> Command, val startingPoseSupplier: (() -> Pose2d)? = null)
}

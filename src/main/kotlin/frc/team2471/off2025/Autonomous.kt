package frc.team2471.off2025

import choreo.Choreo
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.off2025.tests.elevatorSetpointTest
import frc.team2471.off2025.tests.joystickTest
import frc.team2471.off2025.tests.sysIDPivot
import frc.team2471.off2025.tests.slipCurrentTest
import frc.team2471.off2025.util.units.asSeconds
import frc.team2471.off2025.util.isRedAlliance
import frc.team2471.off2025.util.round
import frc.team2471.off2025.util.runOnce
import frc.team2471.off2025.util.sequenceCommand
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import kotlin.collections.forEach
import kotlin.io.path.listDirectoryEntries
import kotlin.io.path.name
import kotlin.jvm.optionals.getOrNull

object Autonomous {
    val paths: MutableMap<String, Trajectory<SwerveSample>> = findChoreoPaths()

    // Chooser for selecting autonomous commands
    private val autoChooser: LoggedDashboardChooser<() -> Command> = LoggedDashboardChooser<() -> Command>("Auto Chooser").apply {
        addOption("8 Foot Straight") { eightFootStraight() }
        addOption("6x6 Square") { squarePathTest() }
        addOption("3 L4 Right") { threeL4Right() }
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
        addOption("Drive Slip Current Test", slipCurrentTest())
        addOption("Armavator Pivot SysId ALL", Armavator.sysIDPivot())
    }

    val autonomousCommand: Command? get() = if (!Drive.demoMode) autoChooser.get()?.invoke() else runOnce { println("DEMO MODE: I'm not running auto, no killing kids today.") }
    val testCommand: Command? get() = testChooser.get()

    /**
     * Initial value determines which side all choreo paths are made for.
     * False = all choreo paths are made on the blue side.
     * True = all choreo paths are made on the red side.
     */
    var isPathsRed = false
        private set

    private var prevPathRed: Boolean? = null

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
    private fun threeL4Right(): Command {
        val path = paths["3 L4 Right"]!!
        return sequenceCommand(
            Drive.driveAlongChoreoPath(path.getSplit(0).get(), poseSupplier = {Drive.localizer.singleTagPose}, resetOdometry = true),
            runOnce { println("L4 Score 1") },
            Drive.driveAlongChoreoPath(path.getSplit(1).get(), poseSupplier = {Drive.localizer.pose}),
            runOnce { println("Intake") },
            Drive.driveAlongChoreoPath(path.getSplit(2).get(), poseSupplier = {Drive.localizer.singleTagPose}),
            runOnce { println("L4 Score 2") },
            Drive.driveAlongChoreoPath(path.getSplit(3).get(), poseSupplier = {Drive.localizer.pose}),
            runOnce { println("Intake") },
            Drive.driveAlongChoreoPath(path.getSplit(4).get(), poseSupplier = {Drive.localizer.singleTagPose}),
            runOnce { println("L4 Score 3") },
        )
    }
}

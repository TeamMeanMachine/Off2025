package frc.team2471.off2025

import choreo.Choreo
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.off2025.tests.joystickTest
import frc.team2471.off2025.util.asSeconds
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

    // Dashboard dropdown chooser for selecting autonomous commands
    private val autoChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("Auto Chooser").apply {
        // Add the 8 ft straight as the default option in the chooser
        addDefaultOption("8 Foot Straight", eightFootStraight())

        // Add the 6x6 square path as an option
        addOption("6x6 Square", squarePathTest())

        // Add the 8 ft spin path as an option
        addOption("8 Foot Spin", eightFootSpin())

        // Add the circle path as an option
        addOption("6 foot circle", circlePathTest())
    }
    private val testChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("Test Chooser").apply {
        // Set up SysId routines and test command options
        addOption("Drive Translation SysId ALL", Drive.sysIDTranslationAll())
        addOption("Drive Rotation SysId ALL", Drive.sysIDRotationAll())
        addOption("Drive Steer SysId ALL", Drive.sysIDSteerAll())
        addOption("Set Angle Offsets", Drive.setAngleOffsets())
        addOption("JoystickTest", joystickTest())
    }
    val autonomousCommand: Command? get() = autoChooser.get()
    val testCommand: Command? get() = testChooser.get()


    private var isPathsRed = false //All paths start blue. Switch to true if all paths made in choreo are on the red side.
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

    // Checks if the alliance color has changed and flips the paths if so
    fun flipPathsIfAllianceChange() {
        if (prevPathRed != null) {
            if (prevPathRed != isRedAlliance) {
                flipPaths()
                isPathsRed = !isPathsRed
            }
        } else if (isRedAlliance != isPathsRed) { //First time through goes here
            flipPaths()
            isPathsRed = !isPathsRed
        }
        prevPathRed = isPathsRed
    }

    // Flip the path so it is correct for the alliance color
    private fun flipPaths() {
        println("flipping paths")
        paths.replaceAll { _, t -> t.flipped() }
        println(paths.map { it.value.sampleAt(0.1, false)?.get()?.chassisSpeeds})
    }

    // Find all the paths in the choreo directory and return a list of them
    fun findChoreoPaths(): MutableMap<String, Trajectory<SwerveSample>> {
        return try {
            val map: MutableMap<String, Trajectory<SwerveSample>> = mutableMapOf()
            Filesystem.getDeployDirectory().toPath().resolve("choreo").listDirectoryEntries("*.traj").forEach {
                val name = it.name.removeSuffix(".traj")
                val traj = Choreo.loadTrajectory(name).getOrNull()
                if (traj != null) {
                    map[name] = traj as Trajectory<SwerveSample>
                }
            }
            println("loaded ${map.size} paths")
            map
        } catch (_: Exception) {
            println("failed to load any auto paths"); mutableMapOf()
        }
    }

    // Run the 8 foot straight path
    fun eightFootStraight(): Command {
        // Simple path used for tuning the drivetrain
        return Drive.driveAlongChoreoPath(paths["8 foot straight"]!!, resetOdometry = true)
    }

    // Run the 6 foot square path
    fun squarePathTest(): Command {
        // Saves the full path to a variable for easy typing
        val path = paths["6 foot square"]!!
        return sequenceCommand(
            // Drive along the first split of the path and reset the robot position to the start of the path
            Drive.driveAlongChoreoPath(path.getSplit(0).get(), resetOdometry = true),

            runOnce { println("finished split 0") }, // Usually a robot action would be run in between splits (like scoring or picking up a game piece)

            // Drive along the second split of the path, we do not need to reset the robot position
            Drive.driveAlongChoreoPath(path.getSplit(1).get(), resetOdometry = false),

            runOnce { println("finished split 1") },

            // Continue to drive along the rest of the path
            Drive.driveAlongChoreoPath(path.getSplit(2).get(), resetOdometry = false),

            runOnce { println("finished split 2") },

            Drive.driveAlongChoreoPath(path.getSplit(3).get(), resetOdometry = false),

            runOnce { println("finished the path") },
        )
    }

    // Run the 8 foot spin path
    fun eightFootSpin(): Command {
        return Drive.driveAlongChoreoPath(paths["8 foot spin"]!!, resetOdometry = true)
    }

    // Run the 6 foot circle path
    fun circlePathTest(): Command {
        return Drive.driveAlongChoreoPath(paths["6 foot circle"]!!, resetOdometry = true)
    }
}

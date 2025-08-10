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

    // Dashboard dropdown chooser for selecting AUTONOMOUS commands.
    private val autoChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("Auto Chooser").apply {
        // Adds the printHelloAuto command as an option on the autonomous chooser.
        addOption("Print Hello Auto", printHelloAuto())

        // Add other auto commands to the chooser here...



    }

    // Dashboard dropdown chooser for TEST commands.
    private val testChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("Test Chooser").apply {
        // Set up SysId routines and test command dashboard options
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

    /*

    AUTONOMOUS COMMANDS GO UNDER HERE

     */


    fun printHelloAuto(): Command {
        return sequenceCommand(
            runOnce { println("Hello I am an autonomous function!") },
            runOnce { println("I am the second command in the sequence!") },
            runOnce { println("Ok bye!") }
        )
    }

    // Create custom driveAlongChoreoPath routines here










}

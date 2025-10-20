package frc.team2471.off2025

import choreo.Choreo
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
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
import org.team2471.frc.lib.control.commands.deferCommand
import org.team2471.frc.lib.control.commands.onlyRunWhileFalse
import org.team2471.frc.lib.control.commands.parallelCommand
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnce
import org.team2471.frc.lib.units.asSeconds
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.control.commands.toCommand
import org.team2471.frc.lib.control.commands.waitCommand
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.meters
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team2471.frc.lib.util.isRedAlliance
import kotlin.collections.forEach
import kotlin.io.path.listDirectoryEntries
import kotlin.io.path.name
import kotlin.jvm.optionals.getOrNull

object Autonomous {
    val paths: MutableMap<String, Trajectory<SwerveSample>> = findChoreoPaths()

    // Chooser for selecting autonomous commands
    private val autoChooser: LoggedDashboardChooser<AutoCommand?> = LoggedDashboardChooser<AutoCommand?>("Auto Chooser").apply {
        addOption("8 Foot Straight", AutoCommand(eightFootStraight()))
        addOption("6x6 Square", AutoCommand(squarePathTest()))
        addOption("3 L4 Right", AutoCommand(threeL4Right()) { Pose2d(7.19158.meters, 3.0.meters, 180.0.degrees.asRotation2d).rotateAroundField { isRedAlliance } })
        addOption("1 L4 Middle", AutoCommand(singleL4Middle()) { Pose2d(7.19158.meters, FieldManager.fieldCenter.y.asMeters.meters, 180.0.degrees.asRotation2d).rotateAroundField { isRedAlliance } })
        addOption("1 L4 ALGAE Middle", AutoCommand(singleL4AlgaeMiddle()) { Pose2d(7.19158.meters, FieldManager.fieldCenter.y.asMeters.meters, 180.0.degrees.asRotation2d).rotateAroundField { isRedAlliance } })
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

    var selectedAuto: AutoCommand? = null
        private set
    val autonomousCommand: Command? get() = if (!Drive.demoMode) selectedAuto?.command else ({ println("DEMO MODE: Not running auto, no killing kids today.") }).toCommand()
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
        readAutoPaths()
    }

    fun updateSelectedAuto() {
        val startTime = RobotController.getMeasureFPGATime()
        val newAuto = autoChooser.get()
        if (selectedAuto != newAuto) {
            selectedAuto = autoChooser.get()
            println("selected auto changed ${autoChooser.sendableChooser.selected}")
            setDrivePositionToAutoStartPose()
            println("finished reading auto in ${(RobotController.getMeasureFPGATime() - startTime).asSeconds} seconds")
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

    fun readAutoPaths() {
        val startTime = RobotController.getMeasureFPGATime()
        val pathNameAndStartPose = mutableListOf<Pair<String, Pose2d>>()
        val segments = mutableListOf<ChassisSpeeds?>()
        paths.forEach {
            pathNameAndStartPose.add(Pair(
                it.value.name(),
                it.value.sampleAt(0.0, false).get().pose
            ))
            val pathSegment = it.value.totalTime / 10.0
            for (i in 0..10) {
                segments.add(it.value.sampleAt(i * pathSegment, false).getOrNull()?.chassisSpeeds)
            }
        }
        println("paths: $pathNameAndStartPose")
        println("reading ${paths.size} paths and ${segments.size} samples. Took ${(RobotController.getMeasureFPGATime() - startTime).asSeconds.round(4)} seconds.")
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
        println(paths.map { it.value.sampleAt(0.0, false)?.get()?.pose})
        readAutoPaths()
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
        return deferCommand(Drive, Armavator) {
            var cycleOne = false
            println("inside L4 right ${Robot.timeSinceEnabled}")
            Drive.pose = Pose2d(7.191587924957275.meters, 3.0.meters, 180.0.degrees.asRotation2d).rotateAroundField { isRedAlliance }
            val coralStationPose = Pose2d(1.508599042892456.meters, 0.7226448655128479.meters, 54.0.degrees.asRotation2d).rotateAroundField { isRedAlliance }//path.getSplit(3).get().getFinalPose(false).get()
            println("about to run sequence ${Robot.timeSinceEnabled}")
            sequenceCommand(
                autoScoreCoral({ if (isRedAlliance) FieldManager.alignPositionsLeftL4Red[2] else FieldManager.alignPositionsLeftL4Blue[2] }, Level.L4).alongWith(
                    runCommand {
                        if (!cycleOne) {
                            println("inside sequence ${Robot.timeSinceEnabled}")
                            cycleOne = true
                        }
                    }.onlyRunWhileFalse { cycleOne }),
                scoreAuto(true),
                parallelCommand(
                    Drive.driveToAutopilotPoint(coralStationPose, { Drive.localizer.odometryPose }, autopilotSupplier = Drive.fastAutoPilot, earlyExit = {_, _ -> Intake.hasCargo && Intake.timeSinceLastScore > 1.5 }),
                    sequenceCommand(
                        runOnce {
                            Drive.resetOdometryToAbsolute()
                        },
                        waitCommand(0.3),
                        runOnce {
                            val isFlipped = FieldManager.getHumanStationAlignHeading(Drive.localizer.pose).second
                            Intake.intakeState = IntakeState.INTAKING
                            Armavator.goToPose(Pose.INTAKE_CORAL_STATION, isFlipped, false)
                        },
                    ),
                    runCommand {
                        Intake.hasCargo = false
                    }.onlyRunWhileFalse {
                        Intake.timeSinceLastScore > 1.5
                    }
                ),
                autoScoreCoral(Level.L4, FieldManager.ScoringSide.RIGHT),
                scoreAuto(true),
                parallelCommand(
                    Drive.driveToAutopilotPoint(coralStationPose, { Drive.localizer.odometryPose }, autopilotSupplier = Drive.fastAutoPilot, earlyExit = {_, _ -> Intake.hasCargo && Intake.timeSinceLastScore > 1.5 }),
                    sequenceCommand(
                        runOnce {
                            Drive.resetOdometryToAbsolute()
                        },
                        waitCommand(0.2),
                        runOnce {
                            val isFlipped = FieldManager.getHumanStationAlignHeading(Drive.localizer.pose).second
                            Intake.intakeState = IntakeState.INTAKING
                            Armavator.goToPose(Pose.INTAKE_CORAL_STATION, isFlipped, false)
                        },
                    ),
                    runCommand {
                        Intake.hasCargo = false
                    }.onlyRunWhileFalse {
                        Intake.timeSinceLastScore > 1.5
                    }
                ),
                autoScoreCoral(Level.L4, FieldManager.ScoringSide.LEFT),
                scoreAuto(),
                runOnce {
                    Armavator.goToPose(Pose(Pose.SCORE_L4.elevatorHeight, 0.0.degrees, Pose.SCORE_L4.pivotAngle))
                    println("finished. Seconds since enable: ${Robot.timeSinceEnabled}")
                }
            )
        }
    }


    private fun singleL4Middle(): Command {
        return deferCommand(Drive, Armavator) {
            println("inside single L4 middle ${Robot.timeSinceEnabled}")
            Drive.pose = Pose2d(7.191587924957275.meters, FieldManager.fieldCenter.y.asMeters.meters, 180.0.degrees.asRotation2d).rotateAroundField { isRedAlliance }
            println("about to run sequence ${Robot.timeSinceEnabled}")
            sequenceCommand (
                autoScoreCoral(Level.L4, FieldManager.ScoringSide.RIGHT, true),
                scoreAuto(waitTime = 1.0),
                runOnce {
                    Armavator.goToPose(Pose(Pose.SCORE_L4.elevatorHeight, 0.0.degrees, Pose.SCORE_L4.pivotAngle))
                    println("finished. Seconds since enable: ${Robot.timeSinceEnabled}")
                }
            )
        }
    }

    private fun singleL4AlgaeMiddle(): Command {
        return deferCommand(Drive, Armavator) {
            println("inside single L4 ALGAE Middle ${Robot.timeSinceEnabled}")
            Drive.pose = Pose2d(7.191587924957275.meters, FieldManager.fieldCenter.y.asMeters.meters, 180.0.degrees.asRotation2d).rotateAroundField { isRedAlliance }
            println("about to run sequence ${Robot.timeSinceEnabled}")
            sequenceCommand (
                autoScoreCoral(Level.L4, FieldManager.ScoringSide.RIGHT, true),
                scoreAuto(waitTime = 1.0),
                runOnce {
                    Armavator.goToPose(Pose(Pose.SCORE_L4.elevatorHeight, 0.0.degrees, Pose.SCORE_L4.pivotAngle))
                    println("finished. Seconds since enable: ${Robot.timeSinceEnabled}")
                },
                waitCommand(1.0),
                parallelCommand(
                    Drive.driveToAutopilotPoint(FieldManager.closestAlignPoint(Drive.localizer.pose, Level.L4, FieldManager.ScoringSide.RIGHT).first.transformBy(Transform2d(-4.0.feet, 0.0.feet, 0.0.degrees.asRotation2d)), autopilotSupplier = Drive.slowAutoPilot),
                    runOnce {
                        goToDrivePose()
                    }
                ),
                algaeDescore().withTimeout(4.0)
            )
        }
    }

    private fun threeL4RightFast(): Command {
        var coralStationPose: Pose2d? = null
        var cycleOne = false
        return sequenceCommand(
            runOnce {
                println("inside L4 right ${Robot.timeSinceEnabled}")
                Drive.pose = Pose2d(7.191587924957275.meters, 3.0.meters, 180.0.degrees.asRotation2d).rotateAroundField { isRedAlliance }
                coralStationPose = Pose2d(1.508599042892456.meters, 0.7226448655128479.meters, 54.0.degrees.asRotation2d).rotateAroundField { isRedAlliance }//path.getSplit(3).get().getFinalPose(false).get()
                println("about to run sequence ${Robot.timeSinceEnabled}")
            },
            sequenceCommand(
                autoScoreCoral({ if (isRedAlliance) FieldManager.alignPositionsLeftL4Red[2] else FieldManager.alignPositionsLeftL4Blue[2] }, Level.L4).alongWith(
                    runCommand {
                        if (!cycleOne) {
                            println("hii ${Robot.timeSinceEnabled}")
                            cycleOne = true
                        }
                    }.onlyRunWhileFalse { cycleOne }),
                scoreAuto(true),
                parallelCommand(
                    Drive.driveToAutopilotPoint({ coralStationPose!! }, { Drive.localizer.odometryPose }, autopilotSupplier = Drive.fastAutoPilot, earlyExit = {_, _ -> Intake.hasCargo && Intake.timeSinceLastScore > 1.5 }).alongWith(),
                    sequenceCommand(
                        runOnce {
                            Drive.resetOdometryToAbsolute()
                        },
                        waitCommand(0.3),
                        runOnce {
                            val isFlipped = FieldManager.getHumanStationAlignHeading(Drive.localizer.pose).second
                            Intake.intakeState = IntakeState.INTAKING
                            Armavator.goToPose(Pose.INTAKE_CORAL_STATION, isFlipped, false)
                        },
                    ),
                    runCommand {
                        Intake.hasCargo = false
                    }.onlyRunWhileFalse {
                        Intake.timeSinceLastScore > 1.5
                    }
                ),
                autoScoreCoral(Level.L4, FieldManager.ScoringSide.RIGHT),
                scoreAuto(true),
                parallelCommand(
                    Drive.driveToAutopilotPoint({ coralStationPose!! }, { Drive.localizer.odometryPose }, autopilotSupplier = Drive.fastAutoPilot, earlyExit = {_, _ -> Intake.hasCargo && Intake.timeSinceLastScore > 1.5 }),
                    sequenceCommand(
                        runOnce {
                            Drive.resetOdometryToAbsolute()
                        },
                        waitCommand(0.3),
                        runOnce {
                            val isFlipped = FieldManager.getHumanStationAlignHeading(Drive.localizer.pose).second
                            Intake.intakeState = IntakeState.INTAKING
                            Armavator.goToPose(Pose.INTAKE_CORAL_STATION, isFlipped, false)
                        },
                    ),
                    runCommand {
                        Intake.hasCargo = false
                    }.onlyRunWhileFalse {
                        Intake.timeSinceLastScore > 1.5
                    }
                ),
                autoScoreCoral(Level.L4, FieldManager.ScoringSide.LEFT),
                scoreAuto(),
                runOnce {
                    Armavator.goToPose(Pose(Pose.SCORE_L4.elevatorHeight, 0.0.degrees, Pose.SCORE_L4.pivotAngle))
                    println("finished. Seconds since enable: ${Robot.timeSinceEnabled}")
                }
            )
        )
    }

    private fun pathPlannerPath(): Command {
        return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("3 L4 Right", 1))
    }

    class AutoCommand(val command: Command, val startingPoseSupplier: (() -> Pose2d)? = null)
}

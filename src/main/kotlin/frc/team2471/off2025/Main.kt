@file:JvmName("Main") // set the compiled Java class name to "Main" rather than "MainKt"
package frc.team2471.off2025

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team2471.off2025.commands.DriveCommands.feedforwardCharacterization
import frc.team2471.off2025.commands.DriveCommands.wheelRadiusCharacterization
import frc.team2471.off2025.commands.ExampleCommand
import frc.team2471.off2025.commands.joystickTest
import frc.team2471.off2025.subsystems.drive.Drive
import frc.team2471.off2025.subsystems.drive.OdometrySignalThread
import frc.team2471.off2025.util.RobotMode
import frc.team2471.off2025.util.robotMode
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import java.net.NetworkInterface

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
object Robot : LoggedRobot() {
    val isCompBot = getCompBotBoolean()

    // Subsystems
    val allSubsystems = arrayOf<Subsystem>(Drive)

    val autonomousCommand: Command? get() = autoChooser.get()
    val testCommand: Command? get() = testChooser.get()

    private var wasDisabled = true

    private val autoChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser("Auto Chooser", AutoBuilder.buildAutoChooser()).apply {
        addOption("ExampleCommand", ExampleCommand())
    }
    private val testChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("Test Chooser").apply {
        // Set up SysId routines
        addOption("Drive Wheel Radius Characterization", wheelRadiusCharacterization())
        addOption("Drive Simple FF Characterization", feedforwardCharacterization())
        addOption("Drive SysId (Quasistatic Forward)", Drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        addOption("Drive SysId (Quasistatic Reverse)", Drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        addOption("Drive SysId (Dynamic Forward)", Drive.sysIdDynamic(SysIdRoutine.Direction.kForward))
        addOption("Drive SysId (Dynamic Reverse)", Drive.sysIdDynamic(SysIdRoutine.Direction.kReverse))
        addOption("Set Angle Offsets", Drive.setAngleOffsets())
        addOption("JoystickTest", joystickTest())
    }

    init {
        when (robotMode) {
            RobotMode.REAL -> { // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }
            RobotMode.SIM -> Logger.addDataReceiver(NT4Publisher()) // Running a physics simulator, log to NT
            RobotMode.REPLAY -> { // Replaying a log, set up replay source
                setUseTiming(false) // Run as fast as possible
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
            }
        }

        DriverStation.silenceJoystickConnectionWarning(true)

        // Start AdvantageKit logger
        Logger.start()
        OdometrySignalThread
        allSubsystems.forEach { println("activating subsystem ${it.name}") }
        OI
    }

    /** This function is called periodically during all modes.  */
    override fun robotPeriodic() {
        // Optionally switch the thread to high priority to improve loop
        // timing (see the template project documentation for details)
//         Threads.setCurrentThreadPriority(true, 99);

        if (Robot.isEnabled) {
            if (wasDisabled) {
                enabledInit()
                wasDisabled = false
            }
        } else {
            wasDisabled = true
        }

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.

        CommandScheduler.getInstance().run()

        // Return to non-RT thread priority (do not modify the first argument)
//         Threads.setCurrentThreadPriority(false, 10);
    }

    /** This function is called once when the robot is enabled.  */
    private fun enabledInit() {
        Drive.brakeMode()
    }

    /** This function is called once when the robot is disabled.  */
    override fun disabledInit() {
        autonomousCommand?.cancel() // This makes sure that the autonomous stops running when teleop starts running.
        testCommand?.cancel()
        Drive.coastMode()
    }

    /** This function is called periodically when disabled.  */
    override fun disabledPeriodic() {}

    /** This function is called once when auto is enabled.  */
    override fun autonomousInit() {
        (autonomousCommand ?: Commands.runOnce({println("THE AUTONOMOUS COMMAND IS NULL")})).schedule() // schedule the autonomous command
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {}

    /** This function is called once when teleop is enabled.  */
    override fun teleopInit() {

    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {}

    /** This function is called once when test mode is enabled.  */
    override fun testInit() {
        CommandScheduler.getInstance().cancelAll() // Cancel all running commands at the start of test mode.
        (testCommand ?: Commands.runOnce({println("THE TEST COMMAND IS NULL")})).schedule()
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {}

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {}

    /** This function is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {}


    private fun getCompBotBoolean(): Boolean {
        var compBot = true
        if (robotMode == RobotMode.REAL) {
            val networkInterfaces =  NetworkInterface.getNetworkInterfaces()
            println("retrieving network interfaces")
            for (iFace in networkInterfaces) {
                println(iFace.name)
                if (iFace.name == "eth0") {
                    println("NETWORK NAME--->${iFace.name}<----")
                    var macString = ""
                    for (byteVal in iFace.hardwareAddress){
                        macString += String.format("%s", byteVal)
                    }
                    println("FORMATTED---->$macString<-----")

                    compBot = (macString == "0-128475710531")
                }
            }
        } else { println("Not real so I am compbot") }
        println("I am compbot = $compBot")
        return compBot
    }
}


/**
 * Main initialization function. Do not perform any initialization here
 * other than calling `RobotBase.startRobot`. Do not modify this file
 * except to change the object passed to the `startRobot` call.
 *
 * If you change the package of this file, you must also update the
 * `ROBOT_MAIN_CLASS` variable in the gradle build file. Note that
 * this file has a `@file:JvmName` annotation so that its compiled
 * Java class name is "Main" rather than "MainKt". This is to prevent
 * any issues/confusion if this file is ever replaced with a Java class.
 * See the [Package Level Functions](https://kotlinlang.org/docs/java-to-kotlin-interop.html#package-level-functions)
 * section on the *Calling Kotlin from Java* page of the Kotlin Docs.
 *
 * If you change your main Robot object (name), change the parameter of the
 * `RobotBase.startRobot` call below to the new name. (If you use the IDE's
 * Rename * Refactoring when renaming the object, it will get changed everywhere
 * including here.)
 */
fun main() = RobotBase.startRobot { Robot }

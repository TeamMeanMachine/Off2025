package frc.team2471.off2025.util

import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.WrapperCommand

/**
 * Decorates this command with a lambda to call on interrupt or end, following the command's inherent end(boolean) method.
 * @param run – A lambda (function) accepting a boolean parameter specifying whether the command was interrupted.
 * @return: the decorated command
 */
fun Command.finallyRun(run: (Boolean) -> Unit): WrapperCommand =
    this.finallyDo { interrupted -> run(interrupted)}

/**
 * Constructs a command that runs an action once and finishes.
 * @param action – the action to run
 * @param requirements – subsystems the action requires
 * @return the command
 * @see edu.wpi.first.wpilibj2.command.InstantCommand
 */
fun runOnceCommand(action: () -> Unit, vararg requirements: Subsystem): Command = Commands.runOnce(action, *requirements)

/**
 * Constructs a command that runs an action every iteration until interrupted.
 * @param action – the action to run
 * @param requirements – subsystems the action requires
 * @return the command
 * @see edu.wpi.first.wpilibj2.command.RunCommand
 */
fun runCommand(action: () -> Unit, vararg requirements: Subsystem): Command = Commands.run(action, *requirements)

/**
 * Runs a group of commands in series, one after the other.
 * @param commands – the commands to include
 * @return the command group
 * @see edu.wpi.first.wpilibj2.command.SequentialCommandGroup
 */
fun sequenceCommand(vararg commands: Command): Command = Commands.sequence(*commands)

/**
 * Runs a group of commands in series, one after the other. Once the last command ends, the group is restarted.
 * @param commands – the commands to include
 * @return the command group
 * @see edu.wpi.first.wpilibj2.command.SequentialCommandGroup,
 * @see Command.repeatedly()
 */
fun repeatingSequenceCommand(vararg commands: Command): Command = Commands.repeatingSequence(*commands)

/**
 * Runs a group of commands at the same time. Ends once all commands in the group finish.
 * @param commands – the commands to include
 * @return the command
 * @see edu.wpi.first.wpilibj2.command.ParallelCommandGroup
 */
fun parallelCommand(vararg commands: Command): Command = Commands.parallel(*commands)

/**
 * Constructs a command that does nothing, finishing after a specified duration.
 * @param seconds – after how long the command finishes
 * @return the command
 * @see edu.wpi.first.wpilibj2.command.WaitCommand
 */
fun waitCommand(seconds: Double): Command = Commands.waitSeconds(seconds)

/**
 * Constructs a command that does nothing, finishing after a specified duration.
 * @param time – after how long the command finishes
 * @return the command
 * @see edu.wpi.first.wpilibj2.command.WaitCommand
 */
fun waitCommand(time: Time): Command = Commands.waitTime(time)

/**
 * Constructs a command that does nothing, finishing once a condition becomes true.
 * @param condition – the condition
 * @return the command
 * @see edu.wpi.first.wpilibj2.command.WaitUntilCommand
 */
fun waitUntilCommand(condition: () -> Boolean): Command = Commands.waitUntil(condition)

/**
 * Constructs a command that does nothing until interrupted.
 * @param requirements – Subsystems to require
 * @return the command
 */
fun idleCommand(vararg requirements: Subsystem) = Commands.idle(*requirements)

/**
 * Runs a group of commands at the same time. Ends once a specific command finishes, and cancels the others.
 * @param deadline – the deadline command
 * @param otherCommands – the other commands to include
 * @return the command group
 * @throws IllegalArgumentException – if the deadline command is also in the otherCommands argument
 * @see edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
 */
fun deadlineCommand(deadline: Command, vararg otherCommands: Command ): Command = Commands.deadline(deadline, *otherCommands)
/**
 * Runs a group of commands at the same time. Ends once any command in the group finishes, and cancels the others.
 * @param commands – the commands to include
 * @return the command group
 * @see edu.wpi.first.wpilibj2.command.ParallelRaceGroup
 */
fun raceCommand(vararg commands: Command): Command = Commands.race(*commands)

/**
 * Runs one of two commands, based on the boolean selector function.
 * @param onTrue – the command to run if the selector function returns true
 * @param onFalse – the command to run if the selector function returns false
 * @param selector – the selector function
 * @return the command
 * @see edu.wpi.first.wpilibj2.command.ConditionalCommand
 */
fun eitherCommand(onTrue: Command, onFalse: Command, selector: () -> Boolean): Command = Commands.either(onTrue, onFalse, selector)

/**
 * Constructs a command that runs an action once, and then runs an action every iteration until interrupted.
 * @param start – the action to run on start
 * @param run – the action to run every iteration
 * @param requirements – subsystems the action requires
 * @return the command
 */
fun startRunCommand(start: () -> Unit, run: () -> Unit, vararg requirements: Subsystem): Command = Commands.startRun(start, run, *requirements)

/**
 * Constructs a command that runs an action once and another action when the command is interrupted.
 * @param start – the action to run on start
 * @param end – the action to run on interrupt
 * @param requirements – subsystems the action requires
 * @return the command
 * @see edu.wpi.first.wpilibj2.command.StartEndCommand
 */
fun startEndCommand(start: () -> Unit, end: () -> Unit, vararg requirements: Subsystem): Command = Commands.startEnd(start, end, *requirements)

/**
 * Constructs a command that runs an action every iteration until interrupted, and then runs a second action.
 * @param run – the action to run every iteration
 * @param end – the action to run on interrupt
 * @param requirements – subsystems the action requires
 * @return the command
 */
fun runEndCommand(run: () -> Unit, end: () -> Unit, vararg requirements: Subsystem): Command = Commands.runEnd(run, end, *requirements)

/**
 * Runs one of several commands, based on the selector function.
 * @param commands – map of commands to select from
 * @param selector – the selector function
 * @param K – The type of key used to select the command
 * @return the command
 * @see edu.wpi.first.wpilibj2.command.SelectCommand
 */
fun <K> selectCommand(commands: Map<K, Command>, selector: () -> K): Command = Commands.select<K>(commands, selector)

/**
 * Runs the command supplied by the supplier.
 * @param supplier – the command supplier
 * @param requirements – the set of requirements for this command
 * @return the command
 * @see edu.wpi.first.wpilibj2.command.DeferredCommand
 */
fun deferCommand(supplier: () -> Command, requirements: MutableSet<Subsystem>): Command = Commands.defer(supplier, requirements)
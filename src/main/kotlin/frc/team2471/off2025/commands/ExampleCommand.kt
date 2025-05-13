package frc.team2471.off2025.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.off2025.subsystems.IntakeSubsystem

/** An example command that uses an example subsystem.  */
class ExampleCommand : Command() {
    init
    {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IntakeSubsystem)
    }

    override fun initialize()
    {

    }

    override fun execute()
    {
        // Called every time the scheduler runs while the command is scheduled.
    }

    override fun end(interrupted: Boolean)
    {
        // Called once the command ends or is interrupted.
    }

    override fun isFinished(): Boolean
    {
        // Returns true when the command should end.
        return false
    }
}

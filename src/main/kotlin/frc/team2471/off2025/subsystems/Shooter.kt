package frc.team2471.off2025.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityDutyCycle
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import org.team2471.frc2025.Falcons

// By making a subsystem a Kotlin object, we ensure there is only ever one instance of it.
// It also reduces the need to have reference variables for the subsystems to be passed around.
object Shooter : SubsystemBase()
{
    /**
     * Example command factory method.
     *
     * @return a command
     */
    fun exampleMethodCommand(): Command = runOnce {
        // Subsystem.runOnce() implicitly add `this` as a required subsystem.
        // TODO: one-time action goes here
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    fun exampleCondition(): Boolean {
        // Query some boolean state, such as a digital sensor.
        return false
    }

    val shooterMotorTop = TalonFX(Falcons.SHOOTER_TOP)

    init {
        shooterMotorTop.configurator.apply(TalonFXConfiguration().apply {
            Slot0.kP = 0.00004
            Slot0.kD = 0.0002
            Slot0.kV = 1.0 / 5800.0
            Slot0.kS = 0.0
        })
    }

    fun setVelocity(rpm: Double) {
        shooterMotorTop.setControl(VelocityDutyCycle(rpm / 60.0))
    }

    override fun periodic()
    {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }

    fun exampleAction()
    {
        // This action is called by the ExampleCommand
        println("ExampleSubsystem.exampleAction has been called")
    }
}
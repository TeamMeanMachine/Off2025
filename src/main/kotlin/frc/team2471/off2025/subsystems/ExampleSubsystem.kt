package frc.team2471.off2025.subsystems

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.off2025.util.logged.LoggedTalonFX

// By making a subsystem a Kotlin object, we ensure there is only ever one instance of it.
// It also reduces the need to have reference variables for the subsystems to be passed around.
object ExampleSubsystem : SubsystemBase() {
    val motor = LoggedTalonFX(2)




    init {
        motor.configSim(DCMotor.getKrakenX60Foc(1), 0.000001)
    }


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

    override fun periodic() {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    fun exampleAction()
    {
        // This action is called by the ExampleCommand
        println("ExampleSubsystem.exampleAction has been called")
    }
}
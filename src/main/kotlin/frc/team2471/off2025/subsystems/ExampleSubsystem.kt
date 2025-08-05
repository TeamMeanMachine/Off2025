package frc.team2471.off2025.subsystems

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.logged.LoggedTalonFX

// By making a subsystem a Kotlin object, we ensure there is only ever one instance of it.
// It also reduces the need to have reference variables for the subsystems to be passed around.
object ExampleSubsystem : SubsystemBase() {
    val motor = LoggedTalonFX(2)

    init {
        println("inside ExampleSubsystem init")
        motor.configSim(DCMotor.getKrakenX60Foc(1), 0.000001)
    }

    override fun periodic() {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    fun exampleAction() {
        // This action is called by the ExampleCommand
        println("ExampleSubsystem.exampleAction has been called")
    }
}
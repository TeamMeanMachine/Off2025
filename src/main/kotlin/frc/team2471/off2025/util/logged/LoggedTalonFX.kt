package frc.team2471.off2025.util.logged

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.team2471.off2025.util.volts

/**
 * Wrapper for that [TalonFX] class that supports simulation when [configSim] is called.
 * @see TalonFX
 * @see DCMotorSim
 */
class LoggedTalonFX(id: Int, canBus: String = ""): TalonFX(id, canBus), LoggedMotor {
    private val talonFXSim = this.simState
    private var motor: DCMotor? = null
    private var motorSim: DCMotorSim? = null


    init {
        MasterMotor.addMotor(this)

        talonFXSim.setSupplyVoltage(12.0.volts)
    }

    /**
     * Configure the simulation to have accurate values.
     * @param motor The type of [DCMotor] motor to sim.
     * @param jKgMetersSquared The moment of inertia of the motor.
     * @see DCMotor
     */
    fun configSim(motor: DCMotor, jKgMetersSquared: Double) {
        this.motor = motor
        motorSim = DCMotorSim(LinearSystemId.createDCMotorSystem(this.motor, jKgMetersSquared, 1.0), this.motor)
        motorSim?.setState(0.0, 0.0)
    }

    override fun simPeriodic() {
        if (motorSim != null) {
            val talonFXVoltage = talonFXSim.motorVoltage

            motorSim!!.inputVoltage = talonFXVoltage
            motorSim!!.update(0.02)

            talonFXSim.setRawRotorPosition(motorSim!!.angularPosition)
            talonFXSim.setRotorVelocity(motorSim!!.angularVelocity)
            talonFXSim.setRotorAcceleration(motorSim!!.angularAcceleration)
        }
    }
}
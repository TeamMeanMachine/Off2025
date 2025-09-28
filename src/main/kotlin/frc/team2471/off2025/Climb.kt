package frc.team2471.off2025

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.units.asDegrees
import frc.team2471.off2025.util.units.rotations
import org.littletonrobotics.junction.Logger

object Climb: SubsystemBase("Climb") {
    val climberMotor = TalonFX(Falcons.CLIMBER_MOTOR)
    val switch = DigitalInput(DigitalSensors.CLIMBER)


    val relayOn: Boolean get() = !switch.get()


    init {
        println("Climb init")

        //TODO: configure climber motor.
        // 20A continuous limit, 30A peak limit, 1 second peak duration
        // coast mode, no invert
        // gear ratio is 45:1
    }


    // Logging periodic. TODO: Set motor power to 0.0 if the relay is active.
    override fun periodic() {
        Logger.recordOutput("Climb/relayOn", relayOn)
        Logger.recordOutput("Climb/motorPower", climberMotor.dutyCycle.valueAsDouble)
        Logger.recordOutput("Climb/motorPositionDeg", climberMotor.position.valueAsDouble.rotations.asDegrees)
    }


    //TODO: needs function that can "deploy" the climber by doing a 180 degree turn (or more) forward. A PID or bang-bang controller should work

    //TODO: needs function to "climb!!!!." Retracts the climber at full power. Should stop when relay is triggered.
}
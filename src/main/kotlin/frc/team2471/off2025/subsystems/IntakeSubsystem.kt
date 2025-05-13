package frc.team2471.off2025.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc2025.Falcons


//Creates a subsystem object
object IntakeSubsystem : SubsystemBase() {
    val intakeMotor = TalonFX(Falcons.INTAKE_MOTOR)

    init {
        //apply a motor configuration
        intakeMotor.configurator.apply(
            //creates a factory default configuration and modifies it
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    //applies a 10 amp limit to motor
                    SupplyCurrentLimit = 10.0
                    SupplyCurrentLimitEnable = true
                }
                MotorOutput.apply {
                    //when no power, brake or coast.
                    NeutralMode = NeutralModeValue.Brake
                }
            }
        )
    }

    fun setPower(percentage: Double) {
        intakeMotor.setControl(DutyCycleOut(percentage))
    }

    fun setVoltage(voltage: Double) {
        intakeMotor.setControl(VoltageOut(voltage))
    }
}
package frc.team2471.off2025.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc2025.CANivores
import org.team2471.frc2025.Falcons

object Elevator: SubsystemBase() {
    //Create motors in elevator
    val motor0 = TalonFX(Falcons.ELEVATOR_0, CANivores.ELEVATOR_CAN)
    val motor1 = TalonFX(Falcons.ELEVATOR_1, CANivores.ELEVATOR_CAN)
    val motor2 = TalonFX(Falcons.ELEVATOR_2, CANivores.ELEVATOR_CAN)
    val motor3 = TalonFX(Falcons.ELEVATOR_3, CANivores.ELEVATOR_CAN)


    val revolutionsPerInch = 1.6


    val heightInches: Double
        get() = motor0.position.valueAsDouble / revolutionsPerInch


    val MIN_HEIGHT_INCHES = 0.0
    val MAX_HEIGHT_INCHES = 58.0

    init {
        val motorConfigs = TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimit = 30.0
                SupplyCurrentLimitEnable = true
            }
            Slot0.apply {
                kP = 0.5
                kG = 0.06
                GravityType = GravityTypeValue.Elevator_Static
            }
            MotorOutput.apply {
                Inverted = InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Coast
            }
            MotionMagic.apply {
                MotionMagicAcceleration = 25.0 * revolutionsPerInch
                MotionMagicCruiseVelocity = 35.0 * revolutionsPerInch
            }
        }

        //apply same configs to each motor
        motor0.configurator.apply(motorConfigs)
        motor1.configurator.apply(motorConfigs)
        motor2.configurator.apply(motorConfigs)
        motor3.configurator.apply(motorConfigs)

        //tell other motors to follow
        motor1.setControl(Follower(motor0.deviceID, true)) //Spins same direction as motor0 to go up
        motor2.setControl(Follower(motor0.deviceID, false)) //Spins opposite direction as motor0 to go up
        motor3.setControl(Follower(motor0.deviceID, false))
    }



    fun setPosition(inches: Double) {
        motor0.setControl(PositionDutyCycle(MathUtil.clamp(inches, MIN_HEIGHT_INCHES, MAX_HEIGHT_INCHES) * revolutionsPerInch))
        println("position: $inches")
    }


    fun setPercentOut(percent: Double) {
        motor0.setControl(DutyCycleOut(percent))
        println("percentage: $percent")
    }


    fun setMotionMagic(inches: Double) {
        motor0.setControl(MotionMagicDutyCycle(MathUtil.clamp(inches, MIN_HEIGHT_INCHES, MAX_HEIGHT_INCHES) * revolutionsPerInch))
    }



}
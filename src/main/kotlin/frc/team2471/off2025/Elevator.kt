package frc.team2471.off2025

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc2025.CANivores
import org.team2471.frc2025.Falcons

object Elevator: SubsystemBase() {

    private val table = NetworkTableInstance.getDefault().getTable("Armavator")
    private val elevatorCurrentEntry = table.getEntry("Elevator Current")
    private val elevatorHeightEntry = table.getEntry("Elevator Motor Height")
    private val elevatorSetpointEntry = table.getEntry("Elevator Setpoint")
    private val elevatorVelocityEntry = table.getEntry("Elevator Velocity")

    //TODO: Create elevator motor
    val motor0 = TalonFX(Falcons.ELEVATOR_0, CANivores.ELEVATOR_CAN)
    val motor1 = TalonFX(Falcons.ELEVATOR_1, CANivores.ELEVATOR_CAN)
    val motor2 = TalonFX(Falcons.ELEVATOR_2, CANivores.ELEVATOR_CAN)
    val motor3 = TalonFX(Falcons.ELEVATOR_3, CANivores.ELEVATOR_CAN)
    val revolutionsPerInch = 1.6


    //TODO: Get height of elevator from motor rotations using a variable getter and the revolutions per inch ratio


    val heightInches: Double
        get() = motor0.position.valueAsDouble / revolutionsPerInch
    var heightSetpoint: Double = 0.0


    val MIN_HEIGHT_INCHES = 0.0
    val MAX_HEIGHT_INCHES = 58.0

    init {
        val motorConfigs = TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimit = 30.0
                SupplyCurrentLimitEnable = true
            }
        MotorOutput.apply {
            Inverted = InvertedValue.Clockwise_Positive
            NeutralMode = NeutralModeValue.Coast
        }
            Slot0.apply {
                kP = 0.5
                kG = 0.06
                GravityType = GravityTypeValue.Elevator_Static
            }
            MotionMagic.apply {
                MotionMagicAcceleration = 25.0 * revolutionsPerInch
                MotionMagicCruiseVelocity = 35.0 * revolutionsPerInch
            }
        }

        motor0.configurator.apply(motorConfigs)
        motor1.configurator.apply(motorConfigs)
        motor2.configurator.apply(motorConfigs)
        motor3.configurator.apply(motorConfigs)


        motor1.setControl(Follower(motor0.deviceID, true))
        motor2.setControl(Follower(motor0.deviceID, false))
        motor3.setControl(Follower(motor0.deviceID, false))
    }

    override fun periodic()
    {
        // This method will be called once per scheduler run
        elevatorHeightEntry.setDouble(heightInches)
        elevatorSetpointEntry.setDouble(heightSetpoint)
        elevatorCurrentEntry.setDouble(motor0.statorCurrent.valueAsDouble)
        elevatorVelocityEntry.setDouble(motor0.velocity.valueAsDouble)
    }

    fun setPercentOut(percent: Double) {
        motor0.setControl(DutyCycleOut(percent))
        println("percentage: $percent")
    }

    fun setPosition(inches: Double) {
        motor0.setControl(
            PositionDutyCycle(
                MathUtil.clamp(
                    inches,
                    MIN_HEIGHT_INCHES,
                    MAX_HEIGHT_INCHES
                ) * revolutionsPerInch
            )
        )
        println("position: $inches")
        heightSetpoint = inches
    }


    //TODO: Make a set motionMagic function that sets position using motionMagic
    fun setMotionMagic(inches: Double) {
        motor0.setControl(
            PositionDutyCycle(
                MathUtil.clamp(
                    inches,
                    MIN_HEIGHT_INCHES,
                    MAX_HEIGHT_INCHES
                ) * revolutionsPerInch
            )
        )
        println("position: $inches")
        heightSetpoint = inches
    }

}
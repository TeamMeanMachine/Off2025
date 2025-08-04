package frc.team2471.off2025.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.LoopLogger
import org.team2471.frc2025.CANivores
import org.team2471.frc2025.Falcons

object Armavator: SubsystemBase() {

    private val table = NetworkTableInstance.getDefault().getTable("Armavator")
    private val elevatorCurrentEntry = table.getDoubleTopic("Elevator Current").publish()
    private val elevatorHeightEntry = table.getDoubleTopic("Elevator Motor Height").publish()
    private val elevatorSetpointEntry = table.getDoubleTopic("Elevator Setpoint").publish()
    private val elevatorVelocityEntry = table.getDoubleTopic("Elevator Velocity").publish()

    private val armCurrentEntry = table.getDoubleTopic("Arm Current").publish()
    private val armAngleEntry = table.getDoubleTopic("Arm Motor Angle").publish()
    private val armSetpointEntry = table.getDoubleTopic("Arm Setpoint").publish()
    private val armVelocityEntry = table.getDoubleTopic("Arm Velocity").publish()

    val elevatorMotor0 = TalonFX(Falcons.ELEVATOR_0, CANivores.ELEVATOR_CAN)
    val elevatorMotor1 = TalonFX(Falcons.ELEVATOR_1, CANivores.ELEVATOR_CAN)
    val elevatorMotor2 = TalonFX(Falcons.ELEVATOR_2, CANivores.ELEVATOR_CAN)
    val elevatorMotor3 = TalonFX(Falcons.ELEVATOR_3, CANivores.ELEVATOR_CAN)

    val armMotor0 = TalonFX(Falcons.ARM_MOTOR_0)
    val armMotor1 = TalonFX(Falcons.ARM_MOTOR_1)

    const val REVOLUTIONS_PER_INCH = 1.6
    const val MIN_HEIGHT_INCHES = 0.0
    const val MAX_HEIGHT_INCHES = 58.0

    const val ARM_GEAR_RATIO = 60.0
    const val MIN_ARM_ANGLE_DEGREES = 0.0
    const val MAX_ARM_ANGLE_DEGREES = 180.0

    inline val currentHeightInches: Double
        get() = elevatorMotor0.position.valueAsDouble / REVOLUTIONS_PER_INCH

    var heightSetpoint: Double = 0.0
        set(value) {
            val safeValue = MathUtil.clamp(value, MIN_HEIGHT_INCHES, MAX_HEIGHT_INCHES)
            elevatorMotor0.setControl(MotionMagicDutyCycle(safeValue * REVOLUTIONS_PER_INCH))
            println("elevator position setpoint: $value")
            field = value
        }

    inline val currentArmAngle: Double
        get() = armMotor0.position.valueAsDouble / ARM_GEAR_RATIO

    var armAngleSetpoint: Double = 0.0
        set(value) {
            val safeValue = MathUtil.clamp(value, MIN_ARM_ANGLE_DEGREES, MAX_ARM_ANGLE_DEGREES)
            armMotor0.setControl(MotionMagicDutyCycle(safeValue * ARM_GEAR_RATIO))
            println("arm angle setpoint: $value")
            field = value
        }

    init {
        println("inside Armavator init")
        val elevatorMotorConfigs = TalonFXConfiguration().apply {
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
                MotionMagicAcceleration = 25.0 * REVOLUTIONS_PER_INCH
                MotionMagicCruiseVelocity = 35.0 * REVOLUTIONS_PER_INCH
            }
        }

        elevatorMotor0.configurator.apply(elevatorMotorConfigs)
        elevatorMotor1.configurator.apply(elevatorMotorConfigs)
        elevatorMotor2.configurator.apply(elevatorMotorConfigs)
        elevatorMotor3.configurator.apply(elevatorMotorConfigs)


        elevatorMotor1.setControl(Follower(elevatorMotor0.deviceID, true))
        elevatorMotor2.setControl(Follower(elevatorMotor0.deviceID, false))
        elevatorMotor3.setControl(Follower(elevatorMotor0.deviceID, false))

        val armMotorConfigs = TalonFXConfiguration().apply { // TODO: tune
            CurrentLimits.apply {
                SupplyCurrentLimit = 30.0
                SupplyCurrentLimitEnable = true
            }
            MotorOutput.apply {
                Inverted = InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Brake
            }
            Slot0.apply {
                kP = 0.5
            }
            MotionMagic.apply {
                MotionMagicAcceleration = 7.0 * 360.0 * ARM_GEAR_RATIO
                MotionMagicCruiseVelocity = 360.0 * ARM_GEAR_RATIO
            }
        }

        armMotor0.configurator.apply(armMotorConfigs)
        armMotor1.configurator.apply(armMotorConfigs)

        armMotor1.setControl(Follower(elevatorMotor0.deviceID, false))
    }

    override fun periodic() {
        LoopLogger.record("b4 Armavator pirdc")
        // This method will be called once per scheduler run
        elevatorHeightEntry.set(currentHeightInches)
        elevatorSetpointEntry.set(heightSetpoint)
        elevatorCurrentEntry.set(elevatorMotor0.statorCurrent.valueAsDouble)
        elevatorVelocityEntry.set(elevatorMotor0.velocity.valueAsDouble)

        armAngleEntry.set(currentArmAngle)
        armSetpointEntry.set(armAngleSetpoint)
        armCurrentEntry.set(armMotor0.statorCurrent.valueAsDouble)
        armVelocityEntry.set(armMotor0.velocity.valueAsDouble)
        LoopLogger.record("Armavator pirdc")
    }

    /**
        percent is a double between -1 and 1
    **/
    fun setElevatorPercentOut(percent: Double) {
        elevatorMotor0.setControl(DutyCycleOut(percent))
        println("elevator percentage: $percent")
    }
}

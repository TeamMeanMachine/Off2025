package frc.team2471.off2025

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
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.LoopLogger
import frc.team2471.off2025.util.asDegrees
import frc.team2471.off2025.util.asInches
import frc.team2471.off2025.util.asRotations
import frc.team2471.off2025.util.degrees
import frc.team2471.off2025.util.rotations
import frc.team2471.off2025.util.wrap
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc2025.CANivores
import org.team2471.frc2025.Falcons

object Armavator: SubsystemBase() {

    private val table = NetworkTableInstance.getDefault().getTable("Armavator")
    private val elevatorCurrentEntry = table.getDoubleTopic("Elevator Current").publish()
    private val elevatorVelocityEntry = table.getDoubleTopic("Elevator Velocity").publish()

    private val armCurrentEntry = table.getDoubleTopic("Arm Current").publish()
    private val armVelocityEntry = table.getDoubleTopic("Arm Velocity").publish()

    val elevatorMotor0 = TalonFX(Falcons.ELEVATOR_0, CANivores.ELEVATOR_CAN)
    val elevatorMotor1 = TalonFX(Falcons.ELEVATOR_1, CANivores.ELEVATOR_CAN)
    val elevatorMotor2 = TalonFX(Falcons.ELEVATOR_2, CANivores.ELEVATOR_CAN)
    val elevatorMotor3 = TalonFX(Falcons.ELEVATOR_3, CANivores.ELEVATOR_CAN)

    val armMotor0 = TalonFX(Falcons.ARM_MOTOR_0, CANivores.ELEVATOR_CAN)
    val armMotor1 = TalonFX(Falcons.ARM_MOTOR_1, CANivores.ELEVATOR_CAN)

    const val REVOLUTIONS_PER_INCH = 1.6
    const val MIN_HEIGHT_INCHES = 0.0
    const val MAX_HEIGHT_INCHES = 58.0

    const val ARM_GEAR_RATIO = 60.0
    const val MIN_ARM_ANGLE_DEGREES = 0.0
    const val MAX_ARM_ANGLE_DEGREES = 180.0

    @get:AutoLogOutput
    inline val currentHeightInches: Double
        get() = elevatorMotor0.position.valueAsDouble / REVOLUTIONS_PER_INCH

    @get:AutoLogOutput
    var heightSetpoint: Double = 0.0
        set(value) {
            val safeValue = MathUtil.clamp(value, MIN_HEIGHT_INCHES, MAX_HEIGHT_INCHES)
            elevatorMotor0.setControl(MotionMagicDutyCycle(safeValue * REVOLUTIONS_PER_INCH))
            println("elevator position setpoint: $value")
            field = value
        }

    @get:AutoLogOutput
    inline val currentArmAngle: Angle
        get() = (armMotor0.position.valueAsDouble / ARM_GEAR_RATIO).rotations

    @get:AutoLogOutput
    var armAngleSetpoint: Angle = 0.0.degrees
        set(value) {
            val safeValue = MathUtil.clamp(value.asDegrees, MIN_ARM_ANGLE_DEGREES, MAX_ARM_ANGLE_DEGREES)
            armMotor0.setControl(MotionMagicDutyCycle(safeValue.degrees.asRotations * ARM_GEAR_RATIO))
            println("arm angle setpoint: ${value.asDegrees}")
            field = value
        }
    val isArmFlipped: Boolean
        get() = currentArmAngle < 0.0.degrees
    val isPivitFlipped: Boolean
        get() = 0.0.degrees.wrap() < 0.0.degrees
    val reverseSpitDirection: Boolean
        get() = isArmFlipped != isPivitFlipped
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
        elevatorCurrentEntry.set(elevatorMotor0.statorCurrent.valueAsDouble)
        elevatorVelocityEntry.set(elevatorMotor0.velocity.valueAsDouble)

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
    fun goToPose(pose: Pose) {
        heightSetpoint = pose.elevatorHeight.asInches
        armAngleSetpoint = pose.armAngle

    }
}
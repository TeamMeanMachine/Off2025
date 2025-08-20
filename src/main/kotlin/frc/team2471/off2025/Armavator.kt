package frc.team2471.off2025

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.LoopLogger
import frc.team2471.off2025.util.units.asDegrees
import frc.team2471.off2025.util.units.asFeetPerSecondPerSecond
import frc.team2471.off2025.util.units.asInches
import frc.team2471.off2025.util.units.asRotations
import frc.team2471.off2025.util.units.cos
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.inches
import frc.team2471.off2025.util.units.feet
import frc.team2471.off2025.util.units.rotations
import frc.team2471.off2025.util.units.sin
import frc.team2471.off2025.util.units.wrap
import org.littletonrobotics.junction.Logger

object Armavator: SubsystemBase() {
    private val table = NetworkTableInstance.getDefault().getTable("Armavator")

    val elevatorMotor0 = TalonFX(Falcons.ELEVATOR_0, CANivores.ELEVATOR_CAN)
    val elevatorMotor1 = TalonFX(Falcons.ELEVATOR_1, CANivores.ELEVATOR_CAN)
    val elevatorMotor2 = TalonFX(Falcons.ELEVATOR_2, CANivores.ELEVATOR_CAN)
    val elevatorMotor3 = TalonFX(Falcons.ELEVATOR_3, CANivores.ELEVATOR_CAN)

    val armMotor0 = TalonFX(Falcons.ARM_MOTOR_0, CANivores.ELEVATOR_CAN)
    val armMotor1 = TalonFX(Falcons.ARM_MOTOR_1, CANivores.ELEVATOR_CAN)

    val pivotMotor = TalonFX(Falcons.PIVOT_MOTOR, CANivores.ELEVATOR_CAN)

    const val ELEVATOR_REVOLUTIONS_PER_INCH = 1.0/1.6
    const val MIN_HEIGHT_INCHES = 0.0
    const val MAX_HEIGHT_INCHES = 58.0

    const val ARM_GEAR_RATIO = 60.0
    const val MIN_ARM_ANGLE_DEGREES = 0.0
    const val MAX_ARM_ANGLE_DEGREES = 180.0

    const val PIVOT_GEAR_RATIO = 360.0 / (33.0 + 1.0/3.0)

    inline val currentHeight: Distance
        get() = (elevatorMotor0.position.valueAsDouble / ELEVATOR_REVOLUTIONS_PER_INCH).inches

    inline val currentArmAngle: Angle
        get() = (armMotor0.position.valueAsDouble / ARM_GEAR_RATIO).rotations

    inline val currentPivotAngle: Angle
        get() = (pivotMotor.position.valueAsDouble / PIVOT_GEAR_RATIO).rotations


    val elevatorFeedforward: Double
        get() = if (currentHeight < 20.0.inches) {
            -0.04
        } else {
            0.04
        }

    const val PIVOT_STATIC_FEED_FORWARD = 0.015
    val pivotFeedForward: Double get() = (0.055 * currentPivotAngle.wrap().sin()) * currentArmAngle.sin()
    val armFeedForward: Double get() = 0.05 * (1.0 + (elevatorMotor0.acceleration.valueAsDouble * ELEVATOR_REVOLUTIONS_PER_INCH / 32.0.feet.asInches)) * -(currentArmAngle + (10.0.degrees * (currentPivotAngle + 90.0.degrees).sin())).sin() +
            0.04 * (Drive.acceleration.rotateBy(-Drive.heading).x.asFeetPerSecondPerSecond / 32.0) * currentArmAngle.cos()


    var heightSetpoint: Distance = 0.0.inches
        set(value) {
            val safeValue = MathUtil.clamp(value.asInches, MIN_HEIGHT_INCHES, MAX_HEIGHT_INCHES)
            elevatorMotor0.setControl(MotionMagicDutyCycle(safeValue * ELEVATOR_REVOLUTIONS_PER_INCH).withFeedForward(elevatorFeedforward))
//            println("elevator position setpoint: $value")
            field = value
        }

    var armAngleSetpoint: Angle = 0.0.degrees
        set(value) {
            val safeValue = MathUtil.clamp(value.asDegrees, MIN_ARM_ANGLE_DEGREES, MAX_ARM_ANGLE_DEGREES)
            armMotor0.setControl(MotionMagicDutyCycle(safeValue.degrees.asRotations * ARM_GEAR_RATIO).withFeedForward(armFeedForward))
//            println("arm angle setpoint: ${value.asDegrees}")
            field = value
        }

    var pivotAngleSetpoint: Angle = 0.0.degrees
        set(value) {
//            pivotMotor.setControl(MotionMagicDutyCycle(value.asRotations * PIVOT_GEAR_RATIO).withFeedForward(pivotFeedForward + PIVOT_STATIC_FEED_FORWARD))
//            println("pivot angle setpoint: ${value.asDegrees}")
            field = value
        }
    val isArmFlipped: Boolean
        get() = currentArmAngle < 0.0.degrees
    val isPivotFlipped: Boolean
        get() = 0.0.degrees.wrap() < 0.0.degrees
    val reverseSpitDirection: Boolean
        get() = isArmFlipped != isPivotFlipped
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
//                kG = 0.06
//                GravityType = GravityTypeValue.Elevator_Static
            }
            MotionMagic.apply {
                MotionMagicAcceleration = 25.0 * ELEVATOR_REVOLUTIONS_PER_INCH
                MotionMagicCruiseVelocity = 35.0 * ELEVATOR_REVOLUTIONS_PER_INCH
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

        Logger.recordOutput("Armavator/currentHeight", currentHeight.asInches)
        Logger.recordOutput("Armavator/currentArmAngle", currentArmAngle.asDegrees)
        Logger.recordOutput("Armavator/currentPivotAngle", currentPivotAngle.asDegrees)

        Logger.recordOutput("Armavator/heightSetpoint", heightSetpoint.asInches)
        Logger.recordOutput("Armavator/armAngleSetpoint", armAngleSetpoint.asDegrees)
        Logger.recordOutput("Armavator/pivotAngleSetpoint", pivotAngleSetpoint.asDegrees)

        Logger.recordOutput("Armavator/elevatorCurrent", elevatorMotor0.supplyCurrent.valueAsDouble)
        Logger.recordOutput("Armavator/armCurrent", armMotor0.supplyCurrent.valueAsDouble)
        Logger.recordOutput("Armavator/pivotCurrent", pivotMotor.supplyCurrent.valueAsDouble)

        Logger.recordOutput("Armavator/elevatorVelocity", elevatorMotor0.velocity.valueAsDouble)
        Logger.recordOutput("Armavator/armVelocity", armMotor0.velocity.valueAsDouble)
        Logger.recordOutput("Armavator/pivotVelocity", pivotMotor.velocity.valueAsDouble)

        Logger.recordOutput("Armavator/elevatorFeedforward", elevatorFeedforward)
        Logger.recordOutput("Armavator/armFeedforward", armFeedForward)
        Logger.recordOutput("Armavator/pivotFeedforward", pivotFeedForward)

        heightSetpoint = heightSetpoint
        armAngleSetpoint = armAngleSetpoint
        pivotAngleSetpoint = pivotAngleSetpoint

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
        heightSetpoint = pose.elevatorHeight
        armAngleSetpoint = pose.armAngle
        pivotAngleSetpoint = pose.pivotAngle
    }
}
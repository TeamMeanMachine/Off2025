package frc.team2471.off2025

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.DynamicMotionMagicDutyCycle
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.CANdi
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.control.LoopLogger
import frc.team2471.off2025.util.control.commands.finallyRun
import frc.team2471.off2025.util.control.commands.onlyRunWhileFalse
import frc.team2471.off2025.util.control.commands.runCommand
import frc.team2471.off2025.util.ctre.addFollower
import frc.team2471.off2025.util.ctre.applyConfiguration
import frc.team2471.off2025.util.ctre.brakeMode
import frc.team2471.off2025.util.ctre.coastMode
import frc.team2471.off2025.util.ctre.currentLimits
import frc.team2471.off2025.util.ctre.d
import frc.team2471.off2025.util.ctre.inverted
import frc.team2471.off2025.util.ctre.magnetSensorOffset
import frc.team2471.off2025.util.ctre.motionMagic
import frc.team2471.off2025.util.ctre.p
import frc.team2471.off2025.util.ctre.remoteCANCoder
import frc.team2471.off2025.util.ctre.s
import frc.team2471.off2025.util.isSim
import frc.team2471.off2025.util.units.absoluteValue
import frc.team2471.off2025.util.units.asDegrees
import frc.team2471.off2025.util.units.asDegreesPerSecond
import frc.team2471.off2025.util.units.asFeetPerSecondPerSecond
import frc.team2471.off2025.util.units.asInches
import frc.team2471.off2025.util.units.asInchesPerSecond
import frc.team2471.off2025.util.units.asRadians
import frc.team2471.off2025.util.units.asRotations
import frc.team2471.off2025.util.units.cos
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.inches
import frc.team2471.off2025.util.units.feet
import frc.team2471.off2025.util.units.perSecond
import frc.team2471.off2025.util.units.rotations
import frc.team2471.off2025.util.units.sin
import frc.team2471.off2025.util.units.unWrap
import frc.team2471.off2025.util.units.wrap
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import motion_profiling.MotionCurve
import org.littletonrobotics.junction.Logger
import kotlin.math.IEEErem
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.min
import kotlin.math.sqrt

object Armavator: SubsystemBase() {
    private val table = NetworkTableInstance.getDefault().getTable("Armavator")

    private val armEncoderOffsetEntry = table.getEntry("Arm Encoder Offset")
    private val elevatorEncoderOffsetEntry = table.getEntry("Elevator Encoder Offset")

    private val pivotEncoderOffsetEntry = table.getEntry("Pivot Encoder Offset")

    val elevatorMotor = TalonFX(Falcons.ELEVATOR_0, CANivores.ELEVATOR_CAN)

    val armMotor = TalonFX(Falcons.ARM_MOTOR_0, CANivores.ELEVATOR_CAN)

    val candi = CANdi(CANSensors.CANDI, CANivores.ELEVATOR_CAN)

    val pivotMotor = TalonFX(Falcons.PIVOT_MOTOR, CANivores.ELEVATOR_CAN)

    inline val rawPivotAngle: Angle
        get() = -candi.pwM1Position.valueAsDouble.IEEErem(1.0).rotations.wrap()

    inline val candiAngle: Angle
        get() = (rawPivotAngle - pivotEncoderOffset.degrees).wrap()

    inline val lampreyAlignmentOffset: Angle
        get() = -abs((kotlin.math.sin(candiAngle.asRadians) * 7.0)).degrees

    inline val pivotEncoderAngle: Angle
        get() = (candiAngle - lampreyAlignmentOffset).wrap()

    inline val rawElevatorEncoderValue: Double
        get() = (elevatorCANcoder.position.valueAsDouble)

    inline val elevatorEncoderHeight: Distance
        get() = elevatorEncoderCurve.getValue(rawElevatorEncoderValue).inches

    inline val pivotMotorAngle: Angle
        get() = (pivotMotor.position.valueAsDouble.rotations / PIVOT_GEAR_RATIO)

    val armCanCoder = CANcoder(CANCoders.ARM, CANivores.ELEVATOR_CAN)

    val elevatorCANcoder = CANcoder(CANCoders.ELEVATOR)
    val elevatorEncoderCurve = MotionCurve()

    const val ELEVATOR_REVOLUTIONS_PER_INCH = 1.0/1.6
    const val MIN_HEIGHT_INCHES = 0.0
    const val MAX_HEIGHT_INCHES = 58.0

    const val ARM_GEAR_RATIO = 60.0
    const val MIN_ARM_ANGLE_DEGREES = -113.0
    const val MAX_ARM_ANGLE_DEGREES = 113.0

    const val PIVOT_GEAR_RATIO = 33.0 + 1.0/3.0
    const val DEFAULT_PIVOT_ACCEL = 25.0 * 360.0
    const val DEFAULT_PIVOT_CRUISING_VEL = 2.0 * 360.0


    inline val currentHeight: Distance
        get() = (elevatorMotor.position.valueAsDouble).inches

    inline val currentArmAngle: Angle
        get() = armMotor.position.valueAsDouble.rotations


    val defaultPivotEncoderOffset =
        if (Robot.isCompBot) 0.0 else -1.841
    val defaultArmEncoderOffset =
        if (Robot.isCompBot) -129.9 else 195.8
    val defaultElevatorEncoderOffset =
        if (Robot.isCompBot) 0.205078125 else 0.205078125

    var pivotEncoderOffset: Double = pivotEncoderOffsetEntry.getDouble(defaultPivotEncoderOffset)
    var armEncoderOffset: Double = armEncoderOffsetEntry.getDouble(defaultArmEncoderOffset)
    var elevatorEncoderOffset: Double = elevatorEncoderOffsetEntry.getDouble(defaultElevatorEncoderOffset)


    val elevatorFeedforward: Double
        get() = if (currentHeight < 20.0.inches) {
            -0.04
        } else {
            0.04
        }

    const val PIVOT_STATIC_FEED_FORWARD = 0.07912
    val pivotFeedForward: Double get() = (0.055 * pivotMotorAngle.wrap().sin()) * currentArmAngle.sin()


    val armFeedForward: Double get() = 0.04 * (1.0 + (elevatorMotor.acceleration.valueAsDouble / 32.0.feet.asInches)) * -(currentArmAngle + (0.0.degrees * (pivotMotorAngle + 90.0.degrees).sin())).sin() +
            0.04 * (Drive.acceleration.rotateBy(-Drive.heading).x.asFeetPerSecondPerSecond / 32.0) * currentArmAngle.cos()


    var heightSetpoint: Distance = 0.0.inches
        set(value) {
            field = MathUtil.clamp(value.asInches, MIN_HEIGHT_INCHES, MAX_HEIGHT_INCHES).inches
            elevatorMotor.setControl(elevatorControlRequest.withPosition(field.asInches).withFeedForward(elevatorFeedforward))
//            println("elevator position setpoint: $value")
        }

    var armAngleSetpoint: Angle = 0.0.degrees
        set(value) {
            field = MathUtil.clamp(value.asDegrees, MIN_ARM_ANGLE_DEGREES, MAX_ARM_ANGLE_DEGREES).degrees
            armMotor.setControl(armControlRequest.withPosition(field.asRotations).withFeedForward(armFeedForward))
//            println("arm angle setpoint: ${value.asDegrees}")
        }

    var pivotAngleSetpoint: Angle = 0.0.degrees
        set(value) {
            field = value.unWrap(pivotMotorAngle)
            pivotMotor.setControl(pivotControlRequest.withPosition(field.asRotations * PIVOT_GEAR_RATIO).withFeedForward(pivotFeedForward * 12.0))
//            println("pivot angle setpoint: ${field.asDegrees}")
        }

    val pivotSetpointError: Angle
        get() = pivotAngleSetpoint - pivotMotorAngle

    val elevatorVelocity: LinearVelocity
        get() = elevatorMotor.velocity.valueAsDouble.inches.perSecond
    val armVelocity: AngularVelocity
        get() = armMotor.velocity.valueAsDouble.rotations.perSecond
    val pivotVelocity: AngularVelocity
        get() = (pivotMotor.velocity.valueAsDouble / PIVOT_GEAR_RATIO).rotations.perSecond

    val noMovement: Boolean
        get() = elevatorVelocity.asInchesPerSecond.absoluteValue < 0.05 &&
                armVelocity.asDegreesPerSecond.absoluteValue < 0.2 &&
                pivotVelocity.asDegreesPerSecond.absoluteValue < 0.2
    val atSetpoint: Boolean
        get() = currentHeight.absoluteValue() - heightSetpoint.absoluteValue() < 0.5.inches &&
                currentArmAngle.absoluteValue() - armAngleSetpoint.absoluteValue() < 0.5.degrees &&
                pivotSetpointError.absoluteValue() - pivotAngleSetpoint.absoluteValue() < 0.5.degrees

    val isArmFlipped: Boolean
        get() = currentArmAngle < 0.0.degrees
    val isPivotFlipped: Boolean
        get() = pivotMotorAngle.wrap() > 0.0.degrees
    val reverseSpitDirection: Boolean
        get() = isArmFlipped != isPivotFlipped

    var periodicFeedForward = true

    // (Cruse velocity, acceleration)
    private val defaultPivotMMSpeeds = Pair(DEFAULT_PIVOT_CRUISING_VEL, DEFAULT_PIVOT_ACCEL)
    private val defaultArmMMSpeeds = Pair(360.0 * ARM_GEAR_RATIO / 40.0, 7.0 * 360.0 * ARM_GEAR_RATIO / 40.0)
    private val defaultElevatorMMSpeeds = Pair(70.0, 300.0)

    private val elevatorControlRequest = DynamicMotionMagicDutyCycle(0.0, defaultElevatorMMSpeeds.first, defaultElevatorMMSpeeds.second, 0.0)
    private val armControlRequest = DynamicMotionMagicDutyCycle(0.0, defaultArmMMSpeeds.first, defaultArmMMSpeeds.second, 0.0)
    private val pivotControlRequest = MotionMagicVoltage(0.0)

    var isSlowSpeed = false
        private set




    init {
        println("inside Armavator init")

        if (!armEncoderOffsetEntry.exists()) {
            armEncoderOffsetEntry.setDouble(defaultArmEncoderOffset)
            println("Shoulder didn't exist!!!!!!!!!!!!!")
        }
        if (!pivotEncoderOffsetEntry.exists()) {
            pivotEncoderOffsetEntry.setDouble(defaultPivotEncoderOffset)
            println("Elbow didn't exist!!!!!!!!!!!!!!!")
        }

        armCanCoder.applyConfiguration {
            inverted(true)
            magnetSensorOffset(armEncoderOffset.degrees.asRotations)
        }

        elevatorCANcoder.applyConfiguration {
            inverted(true)
            magnetSensorOffset(elevatorEncoderOffset)
        }

        // found that it is about 5.8 inches per rotation
        elevatorEncoderCurve.storeValue(0.0, 0.0)
        elevatorEncoderCurve.storeValue(0.305, 1.53)
        elevatorEncoderCurve.storeValue(2.544, 14.85)
        elevatorEncoderCurve.storeValue(3.41, 19.944)
        elevatorEncoderCurve.storeValue(3.87, 22.63)
        elevatorEncoderCurve.storeValue(4.728, 27.60)
        elevatorEncoderCurve.storeValue(5.643, 32.87)
        elevatorEncoderCurve.storeValue(9.99, 62.31)
        elevatorEncoderCurve.storeValue(8.175, 49.233)
        elevatorEncoderCurve.storeValue(6.237, 38.142)

        resetPivot()


        elevatorMotor.applyConfiguration {
            p(0.5 * ELEVATOR_REVOLUTIONS_PER_INCH)

            currentLimits(30.0, 50.0, 0.5)
            inverted(true)
            coastMode()

            motionMagic(defaultElevatorMMSpeeds.first, defaultElevatorMMSpeeds.second)
            remoteCANCoder(elevatorCANcoder.deviceID, 5.8 / ELEVATOR_REVOLUTIONS_PER_INCH, 1.0/5.8)
        }

        elevatorMotor.addFollower(Falcons.ELEVATOR_1, true)
        elevatorMotor.addFollower(Falcons.ELEVATOR_2, false)
        elevatorMotor.addFollower(Falcons.ELEVATOR_3, true)

        armMotor.applyConfiguration {
            p(8.3333)
            d(0.69)

            currentLimits(30.0, 40.0, 1.0)
            inverted(false)
            brakeMode()

            motionMagic(defaultArmMMSpeeds.first, defaultArmMMSpeeds.second)
            remoteCANCoder(CANCoders.ARM, ARM_GEAR_RATIO)
        }

        armMotor.addFollower(Falcons.ARM_MOTOR_1)

        pivotMotor.applyConfiguration {
            p(5.0)
            d(0.2)
            s(PIVOT_STATIC_FEED_FORWARD, StaticFeedforwardSignValue.UseClosedLoopSign)

            currentLimits(20.0, 30.0, 1.0)
            brakeMode()

            motionMagic(defaultPivotMMSpeeds.first, defaultPivotMMSpeeds.second)
        }
    }

    override fun periodic() {
        LoopLogger.record("b4 Armavator pirdc")

        Logger.recordOutput("Armavator/currentHeight", currentHeight.asInches)
        Logger.recordOutput("Armavator/currentArmAngle", currentArmAngle.asDegrees)
        Logger.recordOutput("Armavator/pivotEncoderAngle", pivotEncoderAngle.asDegrees)
        Logger.recordOutput("Armavator/pivotMotorAngle", pivotMotorAngle.asDegrees)

        LoopLogger.record("section 1")

        Logger.recordOutput("Armavator/heightSetpoint", heightSetpoint.asInches)
        Logger.recordOutput("Armavator/armAngleSetpoint", armAngleSetpoint.asDegrees)
        Logger.recordOutput("Armavator/pivotAngleSetpoint", pivotAngleSetpoint.asDegrees)

        LoopLogger.record("section 2")

        Logger.recordOutput("Armavator/elevatorCurrent", elevatorMotor.supplyCurrent.valueAsDouble)
        Logger.recordOutput("Armavator/armCurrent", armMotor.supplyCurrent.valueAsDouble)
        Logger.recordOutput("Armavator/pivotCurrent", pivotMotor.supplyCurrent.valueAsDouble)

        LoopLogger.record("section 3")

        if (!Robot.isAutonomousEnabled) {
            Logger.recordOutput("Armavator/elevatorVelocity", elevatorMotor.velocity.valueAsDouble)
            Logger.recordOutput("Armavator/armVelocity", armMotor.velocity.valueAsDouble)
            Logger.recordOutput("Armavator/pivotVelocity", pivotMotor.velocity.valueAsDouble)
        }


        LoopLogger.record("section 3.5")

        Logger.recordOutput("Armavator/noMovement", noMovement)
        Logger.recordOutput("Armavator/atSetpoint", atSetpoint)

        LoopLogger.record("section 4")
        Logger.recordOutput("Armavator/periodicFeedForward", periodicFeedForward)
        Logger.recordOutput("Armavator/elevatorFeedforward", elevatorFeedforward)
        Logger.recordOutput("Armavator/armFeedforward", armFeedForward)
        Logger.recordOutput("Armavator/pivotFeedforward", pivotFeedForward)

        Logger.recordOutput("Armavator/isSlowSpeed", isSlowSpeed)

//        Logger.recordOutput("Armavator/elevatorError", elevatorMotor.closedLoopError.valueAsDouble)

        LoopLogger.record("section 5")

        if ((pivotMotorAngle.wrap() - pivotEncoderAngle.wrap()).absoluteValue() > 1.0.degrees && pivotVelocity.asDegreesPerSecond.absoluteValue < 0.5 && Robot.isCompBot) {
            resetPivot()
        }

        LoopLogger.record("section 6")
        if (periodicFeedForward) {
            heightSetpoint = heightSetpoint
            armAngleSetpoint = armAngleSetpoint
            pivotAngleSetpoint = pivotAngleSetpoint
        }

//        LoopLogger.record("section 7")
        if (Robot.isDisabled) {
            if (pivotEncoderOffset != pivotEncoderOffsetEntry.getDouble(defaultPivotEncoderOffset)) {
                pivotEncoderOffset = pivotEncoderOffsetEntry.getDouble(defaultPivotEncoderOffset)
                println("pivotEncoderOffset update to $pivotEncoderOffset")
            }
        }

        LoopLogger.record("Armavator pirdc")
    }

    /**
    percent is a double between -1 and 1
     **/
    fun setElevatorPercentOut(percent: Double) {
        elevatorMotor.setControl(DutyCycleOut(percent))
        println("elevator percentage: $percent")
    }

    fun goToPose(pose: Pose, isFlipped: Boolean = false, optimizePivot: Boolean = true) {
        val targetPose = if (isFlipped) pose.reflect() else pose

        if (optimizePivot) {
            if ((targetPose.pivotAngle - pivotEncoderAngle).asDegrees > 90.0) {
                targetPose.pivotAngle -= 180.0.degrees
            } else if ((targetPose.pivotAngle - pivotEncoderAngle).asDegrees < -90.0) {
                targetPose.pivotAngle += 180.0.degrees
            }
        }

        heightSetpoint = targetPose.elevatorHeight
        armAngleSetpoint = targetPose.armAngle
        pivotAngleSetpoint = targetPose.pivotAngle
    }

    fun goToPose(pose: Pose, isFlipped: Boolean = false, optimizePivot: Boolean = true, minimumHeight: Distance, intermediateArmAngle: Angle? = null) = goToPose(pose, { isFlipped }, optimizePivot, minimumHeight, intermediateArmAngle)
    fun goToPose(pose: Pose, isFlipped: () -> Boolean = { false }, optimizePivot: Boolean = true, minimumHeight: Distance, intermediateArmAngle: Angle? = null): Command {
        var targetPose = Pose.DRIVE
        var startPose = Pose.current
        return runOnce {
            targetPose = pose
            startPose = Pose.current
        }.andThen(runCommand(Armavator) { // Intermediate Pose
            if (intermediateArmAngle != null) {
                goToPose(Pose(targetPose.elevatorHeight, intermediateArmAngle, targetPose.pivotAngle), isFlipped(), optimizePivot)
            } else {
                goToPose(Pose(targetPose.elevatorHeight, Pose.current.armAngle, targetPose.pivotAngle))
            }
        }.onlyRunWhileFalse {
            currentHeight.asInches.absoluteValue > minimumHeight.asInches || isSim
        }).andThen(runOnce { // Final Pose
            goToPose(targetPose, isFlipped(), optimizePivot)
        })
    }

    fun animateToPose(pose: Pose, isFlipped: () -> Boolean = { false }, optimizePivot: Boolean = true, animateTime: Double? = null): Command {
        val initialElevatorVelocity = elevatorControlRequest.Velocity
        val initialElevatorAccel = elevatorControlRequest.Acceleration
        val initialArmVelocity = armControlRequest.Velocity
        val initialArmAccel = armControlRequest.Acceleration
        return runOnce {
            println("animating to pose")
            val targetPose = if (isFlipped()) pose.reflect() else pose
            val currentPose = Pose.current

            val elevatorDelta = targetPose.elevatorHeight - currentPose.elevatorHeight
            val armDelta = targetPose.armAngle - currentPose.armAngle


            val elevatorAnimationTime = calculateAnimationTime(elevatorDelta.asInches, initialElevatorVelocity, initialElevatorAccel)
            val armAnimationTime = calculateAnimationTime(armDelta.asDegrees, initialArmVelocity, initialArmAccel)
            val animationTime = maxOf(elevatorAnimationTime, armAnimationTime, animateTime ?: 0.0)

            val elevatorSpeedFactor = min(elevatorAnimationTime / animationTime, 1.0)
            val armSpeedFactor = min(armAnimationTime / animationTime, 1.0)

            elevatorControlRequest.apply {
                Velocity *= elevatorSpeedFactor
                Acceleration *= elevatorSpeedFactor
            }
            armControlRequest.apply {
                Velocity *= armSpeedFactor
                Acceleration *= armSpeedFactor
            }

            println("max animation time: $animationTime. Arm: $armSpeedFactor. Elevator: $elevatorSpeedFactor")

        }.andThen(run {
            goToPose(pose, isFlipped(), optimizePivot)
        }.until {
            atSetpoint || isSim
        }).finallyRun {
            println("stopping armavator animation")
            println("soon-to-be elevator vel $initialElevatorVelocity, accel $initialElevatorAccel")
            println("current elevator vel ${elevatorControlRequest.Velocity}, accel ${elevatorControlRequest.Acceleration}")
            elevatorControlRequest.apply {
                Velocity = initialElevatorVelocity
                Acceleration = initialElevatorAccel
            }
            armControlRequest.apply {
                Velocity = initialArmVelocity
                Acceleration = initialArmAccel
            }
        }
    }

    private fun calculateAnimationTime(distance: Double, maxVelocity: Double, maxAcceleration: Double): Double {
        val minDistanceToReachMaxVelocity = (maxVelocity * maxVelocity) / maxAcceleration
        return if (distance >= minDistanceToReachMaxVelocity) {
            val accelTime = maxVelocity / maxAcceleration
            val accelDistance = 0.5 * maxAcceleration * accelTime * accelTime
            val cruiseDistance = distance - 2.0 * accelDistance
            val cruiseTime = cruiseDistance / maxVelocity

            2.0 * accelTime + cruiseTime
        } else {
            val peakVelocity = sqrt(maxAcceleration * distance)
            val accelTime = peakVelocity / maxAcceleration

            2.0 * accelTime
        }
    }

    fun slowSpeed() {
        if (isSlowSpeed) return // end early
        armControlRequest.apply {
            Velocity = defaultArmMMSpeeds.first * 0.5
            Acceleration = defaultArmMMSpeeds.second * 0.5
        }
        println("applying slow speed")
        isSlowSpeed = true
    }

    fun normalSpeed() {
        if (!isSlowSpeed) return // end early
        armControlRequest.apply {
            Velocity = defaultArmMMSpeeds.first
            Acceleration = defaultArmMMSpeeds.second
        }
        println("applying normal speed")
        isSlowSpeed = false
    }

    fun resetPivot() {
        if (candi.isConnected) {
            println("resetting pivot")
            GlobalScope.launch {
                pivotMotor.setPosition((pivotEncoderAngle.unWrap(pivotMotorAngle) * PIVOT_GEAR_RATIO))
            }
        }
    }
}
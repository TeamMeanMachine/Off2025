package frc.team2471.off2025.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityDutyCycle
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import org.team2471.frc2025.Falcons

// By making a subsystem a Kotlin object, we ensure there is only ever one instance of it.
// It also reduces the need to have reference variables for the subsystems to be passed around.

object Shooter : SubsystemBase() {

    // TODO: Create a network table with the name of the shooter subsystem
    val table = NetworkTableInstance.getDefault().getTable("Shooter")
    // TODO: Create 3 entries.  One for shooter velocity, one for the velocity setpoint, and a third for the voltage applied
    val shooterVelocityEntry = table.getEntry("Velocity")
    val shooterVelocitySetpointEntry = table.getEntry("Velocity Setpoint")
    val shooterVoltsEntry = table.getEntry("Volts")

    // TODO: declare a TalonFX motor controller for the top shooter motor
    val shooterMotorTop = TalonFX(Falcons.SHOOTER_TOP)

    // TODO: declare a const val for MAX_VELOCITY and set it to 4000
    const val MAX_VELOCITY = 4000.0

    // TODO: create a var for the shooterVelocitySetpoint, include a set() function which sets the motors velocity by using VelocityVoltage mode
    var shooterVelocitySetpoint: Double = 0.0
        set(value) {
            shooterMotorTop.setControl(VelocityVoltage(value/60.0))
            field = value
        }

    // TODO: create a val for retrieving the velocity of the shooter motor, include a get() function to evaluate the velocity liveconvert from
    // TODO: be sure to convert CTRE's native units of Rotations per Second to ours Rotations per Minute
    val shooterMotorTopVelocity
        get() = shooterMotorTop.velocity.valueAsDouble * 60.0


    init {
        // TODO: configure the top motor with current limits of 30 amps, include kP, kD, and kV values for our PID.  They can start at 0.
        shooterMotorTop.configurator.apply(TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimit = 30.0
                SupplyCurrentLimitEnable = true
            }
            Slot0.kP = 0.144
            Slot0.kD = 0.0072
            Slot0.kV = 0.116
        })
    }

    // TODO: make a setVoltage function for controlling the motor open loop, like Lesson 1
    fun setVoltage(volts: Double) {
        println("setting voltage $volts")
        shooterMotorTop.setControl(VoltageOut(volts))
    }

    // TODO: make a stop function which uses your setVoltage function to stop the motor
    fun stop() {
        println("stopping")
        setVoltage(0.0)
    }

    override fun periodic() {
        // TODO: set the 3 network table entry values here periodically, so they are all up to date
        shooterVoltsEntry.setDouble(shooterMotorTop.motorVoltage.valueAsDouble)
        shooterVelocityEntry.setDouble(shooterMotorTopVelocity)
        shooterVelocitySetpointEntry.setDouble(shooterVelocitySetpoint)
    }
}
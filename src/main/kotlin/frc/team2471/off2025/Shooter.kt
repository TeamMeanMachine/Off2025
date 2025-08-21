package frc.team2471.off2025

import edu.wpi.first.wpilibj2.command.SubsystemBase

object Shooter : SubsystemBase() {

    // TODO: Create a network table with the name of the shooter subsystem
    // val table = NetworkTableInstance.getDefault().getTable("Shooter")

    // TODO: Create 3 entries.  One for shooter velocity, one for the velocity setpoint, and a third for the voltage applied


    // TODO: declare a TalonFX motor controller for the top shooter motor


    // TODO: declare a const val for MAX_VELOCITY and set it to 4000


    // TODO: create a var for the shooterVelocitySetpoint, include a set() function which sets the motors velocity by using VelocityVoltage mode


    // TODO: create a val for retrieving the velocity of the shooter motor, include a get() function to evaluate the velocity liveconvert from
    // TODO: be sure to convert CTRE's native units of Rotations per Second to ours Rotations per Minute


    init {
        val temp = "" // todo: delete this line

        // TODO: configure the top motor with current limits of 30 amps, include kP, kD, and kV values for our PID.  They can start at 0.
//        shooterMotorTop.configurator.apply(TalonFXConfiguration().apply {
//
//        })
    }

    // TODO: make a setVoltage function for controlling the motor open loop, like Lesson 1
    fun setVoltage(volts: Double) {

    }

    // TODO: make a stop function which uses your setVoltage function to stop the motor


    override fun periodic() {
        // TODO: set the 3 network table entry values here periodically, so they are all up to date

    }
}
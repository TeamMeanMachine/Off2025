package frc.team2471.off2025

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController

object OI {
    // Controller
    private val driverController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)



    init {
        //When driver presses a button set Intake power to 0.9
        driverController.a().onTrue(
            Commands.runOnce({IntakeSubsystem.setPower(0.9)})
        )

        //When driver presses b button set Intake power to 0.0
        driverController.b().onTrue(
            Commands.runOnce({IntakeSubsystem.setPower(0.0)})
        )

        //Use driver left and right triggers to set any power between -1.0 to 1.0. Set to 0.0 if not pressing triggers.
        driverController.leftTrigger(0.1).or(driverController.rightTrigger(0.1)).whileTrue(
            Commands.run({ IntakeSubsystem.setVoltage((driverController.leftTriggerAxis - driverController.rightTriggerAxis) * 12.0) })
                .finallyDo(Runnable { IntakeSubsystem.setVoltage(0.0) })
        )

        //percent output triggers
//        driverController.leftTrigger(0.1).or(driverController.rightTrigger(0.1)).whileTrue(
//            Commands.run({ IntakeSubsystem.setPower(driverController.leftTriggerAxis - driverController.rightTriggerAxis) })
//                .finallyDo(Runnable { IntakeSubsystem.setPower(0.0) })
//        )



    }
}
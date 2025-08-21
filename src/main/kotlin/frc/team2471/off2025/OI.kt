package frc.team2471.off2025

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController

object OI {
    // Controller
    private val driverController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)



    init {

        //TODO: Use left and right triggers to run percent output
        driverController.rightTrigger(0.1).or(driverController.leftTrigger(0.1)).whileTrue(
            Commands.run({Elevator.setPercentOut(MathUtil.applyDeadband(
                driverController.rightTriggerAxis - driverController.leftTriggerAxis, 0.1))})
                .finallyDo(Runnable { Elevator.setPercentOut(0.0) })
        )

        driverController.povUp().onTrue(
         Commands.runOnce({
             println("going up ${Elevator.heightInches}")
             Elevator.setPosition(Elevator.heightInches + 1.0)
         })
      )
      driverController.povDown().onTrue(
          Commands.runOnce({
              println("going down ${Elevator.heightInches}")
              Elevator.setPosition(Elevator.heightInches - 1.0)
          })
      )
        //TODO: down Dpad -> move down 1 inch

        //TODO: use motionMagic to move up/down 10 inches smoothly using left/right dPad
        driverController.povRight().onTrue(
            Commands.runOnce({
                println("going up ${Elevator.heightInches}")
                Elevator.setMotionMagic(Elevator.heightInches + 10.0)
            })
        )
        driverController.povLeft().onTrue(
            Commands.runOnce({
                println("going down ${Elevator.heightInches}")
                Elevator.setMotionMagic(Elevator.heightInches - 10.0)
            })
        )


    }
}
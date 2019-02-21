/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.ArmPreset;
import frc.robot.commands.cargo_grabber.GrabCargo;
import frc.robot.commands.cargo_intake.DeployIntake;
import frc.robot.commands.cargo_intake.IntakeMotorIn;
import frc.robot.commands.hatch.GrabHatch;
import frc.robot.commands.jester_arm.JesterPickupCargo;
import frc.robot.commands.jester_arm.MoveArmTo;

public class PickUpCargo extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PickUpCargo() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    addSequential(new MoveArmTo(ArmPreset.CARGO));
    addSequential(new DeployIntake());
    addSequential(new WaitCommand(0.5));
    addSequential(new MoveArmTo(ArmPreset.PICKUP_CARGO_FLOOR));
    // addSequential(new JesterPickupCargo());
    addSequential(new GrabCargo());
    // addSequential(new GrabHatch());
    // addSequential(new GrabCargo());
    
  }
}

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
import frc.robot.commands.jester_arm.MoveArmTo;
import frc.robot.commands.jester_arm.SetBotState;
import frc.robot.subsystems.JesterArm.BotState;

public class PickUpCargo extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PickUpCargo() {

    addSequential(new SetBotState(BotState.BALL));
    addSequential(new MoveArmTo(ArmPreset.CARGO));
    addParallel(new GrabCargo());
    addSequential(new DeployIntake());
    addSequential(new WaitCommand(0.45));
    addSequential(new MoveArmTo(ArmPreset.PICKUP_CARGO_FLOOR));
    addSequential(new GrabCargo());

  }
}

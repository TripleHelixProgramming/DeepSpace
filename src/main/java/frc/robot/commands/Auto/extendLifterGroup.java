/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.ArmPreset;
import frc.robot.commands.hatch.GrabHatch;
import frc.robot.commands.jester_arm.MoveArmTo;
import frc.robot.commands.jester_arm.SetBotState;
import frc.robot.commands.pid_lifter.ExtendLifter;
import frc.robot.subsystems.JesterArm.BotState;

public class extendLifterGroup extends CommandGroup {
  /**
   * Add your docs here.
   */
  public extendLifterGroup() {
    addSequential(new SetBotState(BotState.HATCH));
    addParallel(new GrabHatch());
    addSequential(new MoveArmTo(ArmPreset.PICKUP_HATCH));
    addSequential(new ExtendLifter());
  }
}

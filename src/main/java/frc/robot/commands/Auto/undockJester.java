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
import frc.robot.commands.cargo_grabber.openGrabber;
import frc.robot.commands.jester_arm.MoveArmTo;

public class undockJester extends CommandGroup {
  /**
   * Add your docs here.
   */
  public undockJester() {
    addParallel(new openGrabber());
    addSequential(new MoveArmTo(ArmPreset.UNPACK_WP1));
    // addSequential(new WaitCommand(0.5));
    // addSequential(new MoveArmTo(ArmPreset.UNPACK_WP2));
    // addSequential(new MoveArmTo(ArmPreset.UNPACK_WP3));
    addSequential(new MoveArmTo(ArmPreset.DELIVER_HATCH_LOWER));
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.ArmPreset;
import frc.robot.commands.cargo_intake.RetractIntake;
import frc.robot.commands.jester_arm.MoveArmTo;

public class resetCargoJester extends CommandGroup {
  /**
   * Add your docs here.
   */
  public resetCargoJester() {
    // Add Commands here:
    addSequential(new MoveArmTo(ArmPreset.FRONT_HATCH_MIDDLE));
    addSequential(new RetractIntake());
  }
}

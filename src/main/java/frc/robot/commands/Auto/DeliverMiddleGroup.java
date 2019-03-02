/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.ArmPreset;
import frc.robot.commands.jester_arm.MoveArmTo;
import frc.robot.commands.jester_arm.SetBotState;
import frc.robot.subsystems.JesterArm;
import frc.robot.subsystems.JesterArm.BotState;
import frc.robot.ArmPreset;

public class DeliverMiddleGroup extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DeliverMiddleGroup() {
    BotState curBotState = JesterArm.getInstance().getState();
    ArmPreset armPos = JesterArm.getInstance().getCurrentArmPreset();
    switch (curBotState) {
    case BALL:
      switch (armPos) {
      case CARGO:
      case DELIVER_BALL_UPPER:
        addSequential(new MoveArmTo(ArmPreset.BALL_TRANSITION_UPPER));
        // addSequential(new MoveArmTo(ArmPreset.DELIVER_BALL_MIDDLE));
        // JesterArm.getInstance().setLastMiddlePos(ArmPreset.DELIVER_BALL_MIDDLE);
        break;
      case DELIVER_BALL_LOWER:
        addSequential(new MoveArmTo(ArmPreset.BALL_TRANSITION_LOWER));
        addSequential(new MoveArmTo(ArmPreset.DELIVER_BALL_MIDDLE));
        JesterArm.getInstance().setLastMiddlePos(ArmPreset.DELIVER_BALL_MIDDLE);
        break;
      }
      break;
    case HATCH:
      addSequential(new MoveArmTo(ArmPreset.DELIVER_HATCH_MIDDLE));
      JesterArm.getInstance().setLastMiddlePos(ArmPreset.DELIVER_HATCH_MIDDLE);
      break;
    case EMPTY:
    default:
      if (JesterArm.getInstance().getLastMiddlePos() == ArmPreset.DELIVER_BALL_MIDDLE) {
        addSequential(new MoveArmTo(ArmPreset.DELIVER_HATCH_MIDDLE));
        JesterArm.getInstance().setLastMiddlePos(ArmPreset.DELIVER_HATCH_MIDDLE);
      } else {
        switch (armPos) {
          case CARGO:
          case DELIVER_BALL_UPPER:
            addSequential(new MoveArmTo(ArmPreset.BALL_TRANSITION_UPPER));
            addSequential(new MoveArmTo(ArmPreset.DELIVER_BALL_MIDDLE));
            JesterArm.getInstance().setLastMiddlePos(ArmPreset.DELIVER_BALL_MIDDLE);
            break;
          case DELIVER_BALL_LOWER:
            addSequential(new MoveArmTo(ArmPreset.BALL_TRANSITION_LOWER));
            addSequential(new MoveArmTo(ArmPreset.DELIVER_BALL_MIDDLE));
            JesterArm.getInstance().setLastMiddlePos(ArmPreset.DELIVER_BALL_MIDDLE);
            break;
          }

      }
      break;
    }
  }

}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.jester_wrist;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ArmPreset;
import frc.robot.subsystems.JesterArm;
import frc.robot.subsystems.JesterWrist;
import frc.robot.subsystems.JesterArm.BotState;

public class FollowArm extends Command {
  public FollowArm() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(JesterWrist.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    HelixEvents.getInstance().addEvent("JESTER_WRIST", "Starting FollowArm");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Set the wrist position based of the arm position.
    
    ArmPreset currentPreset = JesterArm.getInstance().getCurrentArmPreset();
    BotState bot_state = JesterArm.getInstance().getState();
    int curArmAngle;

    //  Don't move wrist until arm is sent a preset.
    if (currentPreset != ArmPreset.START) {
        // Get angle cooresponding to current arm sensor position.
        curArmAngle = currentPreset.CalcArmAngle(JesterArm.getInstance().getArmPos());
        if (((curArmAngle > 120) && (curArmAngle < 180)) && bot_state == BotState.BALL) {
          // JesterWrist.getInstance().setWristMotionMagic(230);
          JesterWrist.getInstance().setWristMotionMagic(495);

        } else {
          JesterWrist.getInstance().setWristPos(currentPreset);
        }
    }
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    HelixEvents.getInstance().addEvent("JESTER_WRIST", "End FollowArm");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    HelixEvents.getInstance().addEvent("JESTER WRIST", "FollowArm() interuppted");
  }
}

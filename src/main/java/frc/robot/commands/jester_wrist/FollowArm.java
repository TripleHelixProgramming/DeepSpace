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
    int curArmAngle, wrist_pos, offset = -68;
    boolean curpos_gt_150, dest_gt_180, dest_lt_180, curpos_gt_90, dest_in_90_to_180;

    //  Don't move wrist until arm is sent a preset.
    if ((currentPreset != ArmPreset.START) && (currentPreset != ArmPreset.MANUAL)) {
        // Get angle cooresponding to current arm sensor position.

        curArmAngle = currentPreset.CalcArmAngle(JesterArm.getInstance().getArmPos());
        curpos_gt_150 = curArmAngle > 150;
        curpos_gt_90 = curArmAngle > 90;
        dest_gt_180 = currentPreset.getShoulderAngle() > 180;
        dest_lt_180 = currentPreset.getShoulderAngle() < 180;
        dest_in_90_to_180 = ((currentPreset.getShoulderAngle() > 90) && (currentPreset.getShoulderAngle() <180));

        if ((curpos_gt_90) && (!JesterArm.getInstance().ArmIsCloseToPreset()) && (dest_gt_180)) {
              wrist_pos = currentPreset.WristAngleToPos(curArmAngle + offset);
              JesterWrist.getInstance().setWristMotionMagic(wrist_pos);
        } else if ((currentPreset.getShoulderAngle() < 180) && (curArmAngle > 150) &&
                  (curArmAngle >  currentPreset.getShoulderAngle()) 
                  && (!JesterArm.getInstance().ArmIsCloseToPreset())) {
              wrist_pos = currentPreset.WristAngleToPos(curArmAngle + offset);
              JesterWrist.getInstance().setWristMotionMagic(wrist_pos);
        } else {
              JesterWrist.getInstance().setWristPos(currentPreset);
        }
        // if (curArmAngle > 90 ){ 
        //   offset = -68; 
        // } else {
        //   offset = 0;
        // }
        // if (JesterArm.getInstance().ArmIsCloseToPreset()) {
        //   JesterWrist.getInstance().setWristPos(currentPreset);
        // } else {             
        //   wrist_pos = currentPreset.WristAngleToPos(curArmAngle + offset);
        //   JesterWrist.getInstance().setWristMotionMagic(wrist_pos);
        // }
        // if (((curArmAngle > 147) && (curArmAngle < 195)) && bot_state == BotState.BALL) {
        //       wrist_pos = currentPreset.WristAngleToPos(curArmAngle-65);
        //       JesterWrist.getInstance().setWristMotionMagic(wrist_pos);  // wrist_pos should be 230 PB 495 CB
        // } else if (((curArmAngle > 150) && (curArmAngle < 160)) && bot_state == BotState.BALL) {
        //       wrist_pos = currentPreset.WristAngleToPos(230);
        //       JesterWrist.getInstance().setWristMotionMagic(wrist_pos);  // wrist_pos should be 230 PB 495 CB
        // } else if (((curArmAngle >= 160) && (curArmAngle < 170)) && bot_state == BotState.BALL) {
        //       wrist_pos = currentPreset.WristAngleToPos(240);
        //       JesterWrist.getInstance().setWristMotionMagic(wrist_pos);  // wrist_pos should be 230 PB 495 CB
        // } else if (((curArmAngle >= 170) && (curArmAngle < 180)) && bot_state == BotState.BALL) {
        //       wrist_pos = currentPreset.WristAngleToPos(250);
        //       JesterWrist.getInstance().setWristMotionMagic(wrist_pos);  // wrist_pos should be 230 PB 495 CB
        // } else if (((curArmAngle >= 180) && (curArmAngle < 200)) && bot_state == BotState.BALL) {
        //       wrist_pos = currentPreset.WristAngleToPos(260);
        //       JesterWrist.getInstance().setWristMotionMagic(wrist_pos);  // wrist_pos should be 230 PB 495 CB
        // } else {
          
        // }
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

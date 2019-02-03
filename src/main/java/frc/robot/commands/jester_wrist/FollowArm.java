/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.jester_wrist;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.JesterArm;
import frc.robot.subsystems.JesterArm.ArmPos;
import frc.robot.subsystems.JesterWrist;
import frc.robot.subsystems.JesterWrist.Wrist;

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

    int arm_pos = JesterArm.getInstance().getArmPos();
    int new_wrist_pos;
    
    SmartDashboard.putNumber("Arm Pos", arm_pos);

    // Caclualate next wrist Pos
    if (arm_pos <= ArmPos.START.pos) { 
        new_wrist_pos = Wrist.START.pos;

    } else if ((arm_pos > ArmPos.START.pos ) && (arm_pos <= ArmPos.FRONT_UPPER.pos)) {
        new_wrist_pos = Wrist.FRONT.pos;

    } else if ((arm_pos > ArmPos.FRONT_UPPER.pos) && (arm_pos < ArmPos.BACK_UPPER.pos)) {
        new_wrist_pos = Wrist.TRANSITION.pos;

    } else {
        new_wrist_pos = Wrist.BACK.pos;
    }

    SmartDashboard.putNumber("Wrist Pos", new_wrist_pos);
    JesterWrist.getInstance().setWristPos(new_wrist_pos);
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

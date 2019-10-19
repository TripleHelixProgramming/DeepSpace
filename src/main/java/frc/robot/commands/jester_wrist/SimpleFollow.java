package frc.robot.commands.jester_wrist;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ArmPreset;
import frc.robot.subsystems.JesterArm;
import frc.robot.subsystems.JesterWrist;
import frc.robot.subsystems.JesterArm.BotState;

public class SimpleFollow extends Command {
  public SimpleFollow() {
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

    //  Don't move wrist until arm is sent a preset.
    if ((currentPreset != ArmPreset.START) && (currentPreset != ArmPreset.MANUAL)) {
        // Get angle cooresponding to current arm sensor position.
        JesterWrist.getInstance().setWristPos(currentPreset);
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

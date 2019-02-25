/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.jester_arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.JesterArm;
import frc.robot.subsystems.JesterArm.BotState;

public class SetBotState extends Command {
  private BotState bot_state;

  public SetBotState(BotState bot_state) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(JesterArm.getInstance());
    this.bot_state = bot_state;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    JesterArm.getInstance().setState(bot_state);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

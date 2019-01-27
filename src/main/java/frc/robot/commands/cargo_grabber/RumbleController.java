/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo_grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;

public class RumbleController extends Command {


  public RumbleController() {
      // Use requires() here to declare subsystem dependencies
      // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    setTimeout(2);
  }

  // Called repeatedly when this Command is scheduled to run
  protected void execute() {
    OI.getInstance().setControllerRumble(true);
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
      return isTimedOut();
  }

  // Called once after isFinished returns true
  protected void end() {
    OI.getInstance().setControllerRumble(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  }
}

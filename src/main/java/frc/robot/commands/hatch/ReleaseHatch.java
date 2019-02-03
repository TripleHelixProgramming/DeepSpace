/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.HatchGrabber;

public class ReleaseHatch extends Command {
  public ReleaseHatch() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(HatchGrabber.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    HelixEvents.getInstance().addEvent("RELEASE_HATCH", "Starting to release hatch");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    HatchGrabber.getInstance().hatchRelease();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return HatchGrabber.getInstance().hasHatch();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    HelixEvents.getInstance().addEvent("RELEASE_HATCH", "Ending release hatch");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

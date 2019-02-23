/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.RumbleController;
import frc.robot.subsystems.CargoGrabber;
import frc.robot.subsystems.HatchGrabber;

public class GrabHatch extends Command {

  Command rumbleCommand = new RumbleController();

  public GrabHatch() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(HatchGrabber.getInstance());
    requires(CargoGrabber.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    HelixEvents.getInstance().addEvent("GRAB_HATCH", "Starting to grab hatch");
    CargoGrabber.getInstance().openGrabber();
    HatchGrabber.getInstance().hatchRelease();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    HatchGrabber.getInstance().hatchGrab();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }
  // Called once after isFinished returns true
  @Override
  protected void end() {
    // HelixEvents.getInstance().addEvent("GRAB_HATCH", "Ending grab hatch");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

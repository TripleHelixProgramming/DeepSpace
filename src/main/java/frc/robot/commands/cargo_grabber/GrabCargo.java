/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo_grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.RumbleController;
import frc.robot.subsystems.CargoGrabber;
import frc.robot.subsystems.HatchGrabber;

import com.team2363.logger.HelixEvents;

public class GrabCargo extends Command {

  private boolean isFinished = false;


  private int stalledCount = 0;
	
  Command rumbleCommand = new RumbleController();
  
  public GrabCargo() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
   requires(HatchGrabber.getInstance());
    requires(CargoGrabber.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   HatchGrabber.getInstance().hatchGrab();
   CargoGrabber.getInstance().closeGrabber();
   HelixEvents.getInstance().addEvent("GRAB_CARGO", "Starting to grab cargo");

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean isFinished = false;

   //when we retrieve cargo, extend hatch grabber
   if (CargoGrabber.getInstance().isOverCurrent()) {
    stalledCount++;
  } else {
    stalledCount = 0;
  }
  CargoGrabber.getInstance().intake();

if (stalledCount > 5) {
  isFinished = true;
  if (!rumbleCommand.isRunning()) {
    rumbleCommand.start();
  }
}

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
    // return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    HelixEvents.getInstance().addEvent("GRAB_CARGO", "Ending grab cargo");
    CargoGrabber.getInstance().stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

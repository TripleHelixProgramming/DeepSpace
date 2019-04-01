/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo_grabber;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoGrabber;

public class stopCargoGrabber extends Command {
  public stopCargoGrabber() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(CargoGrabber.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    HelixEvents.getInstance().addEvent("STOP CARGO GRABBER", "Starting to stop cargo grabber");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    CargoGrabber.getInstance().stopMotors();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    HelixEvents.getInstance().addEvent("STOP CARGO GRABBER", "Ending to stop cargo grabber");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

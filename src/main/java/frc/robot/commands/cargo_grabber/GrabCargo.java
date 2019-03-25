/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo_grabber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.RumbleController;
import frc.robot.subsystems.CargoGrabber;
import frc.robot.subsystems.HatchGrabber;
import frc.robot.subsystems.JesterArm;
import frc.robot.subsystems.JesterArm.BotState;

import com.team2363.logger.HelixEvents;

public class GrabCargo extends Command {

  private boolean finished = false;


  private int stalledCount = 0;
  private double speed = 0.40;
	
  Command rumbleCommand = new RumbleController();
  
  public GrabCargo() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
   requires(HatchGrabber.getInstance());
    requires(CargoGrabber.getInstance());
    requires(JesterArm.getInstance());
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
  JesterArm.getInstance().setState(BotState.BALL);

  finished = false;

   //when we retrieve cargo, extend hatch grabber
   if (CargoGrabber.getInstance().isOverCurrent()) {
    stalledCount++;
    SmartDashboard.putNumber("Cargo Stall Count", stalledCount);
  } else {
    stalledCount = 0;
    SmartDashboard.putNumber("Cargo Stall Reset", stalledCount);
  }
  CargoGrabber.getInstance().intake(speed);

if (stalledCount > 15) {
  finished = true;
  CargoGrabber.getInstance().slowMotors();
  if (!rumbleCommand.isRunning()) {
    rumbleCommand.start();
  }
}

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
    // return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("Grab Cargo", "Ending grab cargo");
    HelixEvents.getInstance().addEvent("GRAB_CARGO", "Ending grab cargo");
    CargoGrabber.getInstance().stopMotors();
    // HatchGrabber.getInstance().hatchRelease();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

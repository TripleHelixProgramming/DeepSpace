/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo_intake;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoIntake;

public class IntakeMotorOut extends Command {
  public IntakeMotorOut() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(CargoIntake.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    HelixEvents.getInstance().addEvent("CARGO_INTAKE", "Starting IntakeMotorOut");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    CargoIntake.getInstance().out();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    HelixEvents.getInstance().addEvent("CARGO_INTAKE", "Ending IntakeMotorOut");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

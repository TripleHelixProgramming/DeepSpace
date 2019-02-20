/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.jester_arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ArmPreset;
import frc.robot.subsystems.CargoGrabber;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.HatchGrabber;
import frc.robot.subsystems.JesterArm;

public class JesterPickupCargo extends Command {
  public JesterPickupCargo() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(JesterArm.getInstance());
    requires(CargoIntake.getInstance());
    requires(CargoGrabber.getInstance());
    requires(HatchGrabber.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      // JesterArm.getInstance().goTo(ArmPreset.PICK_UP);
      HatchGrabber.getInstance().hatchGrab();      
      CargoGrabber.getInstance().intake();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Camera;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;

public class seekingByVision extends Command {
  private double kpAim = 0.05;
  private double throttleInput;
  private double turnInput ;

  public seekingByVision() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Camera.getInstance());
    requires(Drivetrain.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // float tv = table->GetNumber("tv");
    boolean targetFound = Camera.getInstance().getIsTargetFoundFront();
    double tx = Camera.getInstance().getdegRotationToTargetFront();

double turnInput = 0.0f;
if (targetFound)
{
        // We do see the target,  execute aiming code.
        // double getdegRotationToTargetFront = tx;
        turnInput = kpAim * tx;
}
else
{
        // We don't see the target, seek for the target by spinning in place at a safe speed
        turnInput = 0.3f;
}

Drivetrain.getInstance().arcadeDrive(throttleInput, turnInput, true);

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

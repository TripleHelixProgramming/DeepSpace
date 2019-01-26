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

public class aimByVision extends Command {
  double Kp = 0.0305;
  double kpDistance = 0.0205;
  double min_command = 0.02; 
  private double throttleInput;
  private double turnInput ;
  private boolean finished = false;

  // private double left_command = 0.0;
  // private double right_command = 0.0;
  
  public aimByVision() {
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
    Camera.getInstance().setDockingMode();
      double txFront = Camera.getInstance().getdegRotationToTargetFront();
      boolean targetFound = Camera.getInstance().getIsTargetFoundFront();
      double heading_error = -txFront;
      double steering_adjust = 0.0;

      if(Math.abs(txFront) < 1) {
        finished = true;
      }
  
      if (txFront > 0.0) {
          steering_adjust = Kp*txFront - min_command;
      }else if (txFront < 0.0){
          steering_adjust = Kp*txFront + min_command;
      }
  
      Drivetrain.getInstance().arcadeDrive(throttleInput, turnInput, true);
    // Camera.getInstance().setDockingMode();
    // double txFront = Camera.getInstance().getdegRotationToTargetFront();
    // double steering_adjust = 0.0;
    
    // if (txFront < 1.0)
    // {
    //         rotateValue = kpAim * rotateValue + txFront;
    // }
    // else if (txFront > 1.0)
    // {
    //         rotateValue = kpAim * rotateValue - txFront;
    // }

    // Drivetrain.getInstance().arcadeDrive(moveValue, rotateValue, false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    Camera.getInstance().setCameraMode();
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

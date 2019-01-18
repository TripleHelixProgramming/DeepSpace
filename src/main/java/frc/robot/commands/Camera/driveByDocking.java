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

public class driveByDocking extends Command {

    private double kpAim = 0.05;
    private double kpDistance = 0.05;
    private double moveValue;
    private double rotateValue ;
    private double left_command = 0.0;
    private double right_command = 0.0;

  public driveByDocking() {
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
    Camera.getInstance().setDockingMode();
    double txFront = Camera.getInstance().getdegRotationToTargetFront();
    double txBack = Camera.getInstance().getdegRotationToTargetBack();
    double tyFront = Camera.getInstance().getdegVerticalToTargetFront();
    double tyBack = Camera.getInstance().getdegVerticalToTargetBack();
    boolean TargetFoundFront = Camera.getInstance().getIsTargetFoundFront();
    boolean TargetFoundBack = Camera.getInstance().getIsTargetFoundBack();

    // if(TargetFoundFront){
    //   moveValue = tyFront * kpDistance;
    //   rotateValue = txFront * kpAim;
    // }else{
    //   moveValue = 0;
    //   rotateValue = 0;
    // }

    // if(TargetFoundBack){
    //   moveValue = tyBack * kpDistance;
    //   rotateValue = txBack * kpAim;
    // }else{
    //   moveValue = 0;
    //   rotateValue = 0;
    // }

    double steering_adjust = 0.0;
    
    if (txFront > 1.0)
    {
            steering_adjust = kpAim * rotateValue + txFront;
    }
    else if (txFront < 1.0)
    {
            rotateValue = kpAim * rotateValue - txFront;
    }

    double distance_adjust = kpDistance * tyFront;
    
    left_command += steering_adjust + distance_adjust;
    right_command -= steering_adjust + distance_adjust;
    
    Drivetrain.getInstance().tankDrive(left_command, right_command);   //Remove the boolean value from arcadeDrive?
    
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

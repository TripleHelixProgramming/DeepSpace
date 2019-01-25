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

    // private double kpAim = 0.05;
    // private double kpDistance = 0.05;
    // private double throttleInput;
    // private double rotateValue ;
    // private double left_command = 0.0;
    // private double right_command = 0.0;
    double Kp = 0.0305;
    double kpDistance = 0.0205;
    double min_command = 0.02;
    double left_command;
    double right_command;
    private boolean finished = false;

  public driveByDocking() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Camera.getInstance());
    requires(Drivetrain.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    left_command = 0.0;
    right_command = 0.0;

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
    double heading_error = -txFront;
    double steering_adjust = 0.0;
    double distance_error = -tyFront; //subtracting 2 due to range error on camera on back of the robot.

    

    if(Math.abs(txFront) < 1) {
      finished = true;
    }

    if (txFront > 0.0) {
        steering_adjust = Kp*txFront - min_command;
    }else if (txFront < 0.0){
        steering_adjust = Kp*txFront + min_command;
    }

    // if(tyFront < 0.0){
    //   left_command = 0.25;
    //   right_command = 0.25;
    
    // }else if(tyFront >0.0){
    //   left_command = 0.0;
    //   right_command =0.0;
    // }
    double distance_adjust = (kpDistance * distance_error);

    left_command += steering_adjust - distance_adjust;
    right_command -= steering_adjust + distance_adjust; //changed from "-"
    
    // Drivetrain.getInstance().tankDrive(left_command, right_command);   //Remove the boolean value from arcadeDrive?
    Drivetrain.getInstance().tankDrive(left_command, right_command);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
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

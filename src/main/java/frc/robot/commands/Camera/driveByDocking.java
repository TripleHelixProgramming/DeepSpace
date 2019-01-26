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
import frc.robot.subsystems.Camera.CAMERA;

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

  private CAMERA location;

  public driveByDocking(CAMERA location) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Camera.getInstance());
    requires(Drivetrain.getInstance());

    this.location = location;
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
    Camera.getInstance().setCamera(location);
    Camera.getInstance().setDockingMode();

    double tx = Camera.getInstance().RotationalDegreesToTarget();
    double ty = Camera.getInstance().VerticalDegreesToTarget();

    double steering_adjust = 0.0;
    double distance_error = -ty; // subtracting 2 due to range error on camera on back of the robot.

    if (Math.abs(tx) < 1) {
      finished = true;
    }

    if (tx > 0.0) {
      steering_adjust = Kp * tx - min_command;
    } else if (tx < 0.0) {
      steering_adjust = Kp * tx + min_command;
    }

    // if (ty < 0.0) {
    //    left_command = 0.25;
    //    right_command = 0.25;
    // } else if (ty > 0.0) {
    //    left_command = 0.0;
    //    right_command =0.0;
    // }

    double distance_adjust = (kpDistance * distance_error);

    left_command += steering_adjust - distance_adjust;
    right_command -= steering_adjust + distance_adjust; // changed from "-"

    // Drivetrain.getInstance().tankDrive(left_command, right_command); //Remove the
    // boolean value from arcadeDrive?
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
      Camera.getInstance().setCameraMode();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
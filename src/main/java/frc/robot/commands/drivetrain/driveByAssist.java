/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.OI;
import frc.robot.camera.CAMERA;

public class driveByAssist extends Command {

  double Kp = 0.03;
  double Ki = 0.0;
  double Kd = 0.50;
  double kpDistance = 0.0215;
  double min_command = 0.0;
  double left_command;
  double right_command;
  double integral;
  double derivative;
  double last_error;
  double result;
  double error;
  private boolean finished = false;

  private CAMERA camera;

  public driveByAssist(CAMERA camera) {

    requires(Drivetrain.getInstance());

    this.camera = camera;
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

    camera.setDockingMode();

    double tx = camera.RotationalDegreesToTarget();
    double ty = camera.VerticalDegreesToTarget();

    double steering_adjust = 0.0;
    double distance_error = -ty;

    if (Math.abs(tx) < 1) {
      finished = true;
    }

    if (tx > 0.0) {
      steering_adjust = Kp * tx - min_command;
    } else if (tx < 0.0) {
      steering_adjust = Kp * tx + min_command;
    }

    double distance_adjust = OI.getInstance().getThrottle() * 0.8;

    if (camera == CAMERA.FRONT) {
      left_command += steering_adjust - distance_adjust;
      right_command -= steering_adjust + distance_adjust; // changed from "-"
    } else {
      left_command += steering_adjust + distance_adjust;
      right_command -= steering_adjust - distance_adjust;
    }

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

  public double PIDCalc2(double error) {
    integral += error * .2;
    derivative = error - last_error;
    last_error = error;
    result = Kp * error + Ki * integral + Kd * derivative;
    return result;
  }

}
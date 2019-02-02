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

public class driveByDockingPID extends Command {

  double Kp = 0.015;
  double Ki = 0.0;
  double Kd = 0.0275;
  // double kpDistance = 0.0275;
  double kpDistance = 0.025;

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

  public driveByDockingPID(CAMERA camera) {

    requires(Drivetrain.getInstance());
    this.camera = camera;
  }

  @Override
  protected void initialize() {
    left_command = 0.0;
    right_command = 0.0;

  }

  @Override
  protected void execute() {

    camera.setDockingMode();
    double tx = camera.RotationalDegreesToTarget();
    double ty = camera.VerticalDegreesToTarget();

    double steering_adjust = 0.0;
    double distance_error = -ty;

    if (Math.abs(tx) < 0.1) {
      finished = true;
    }

    steering_adjust = PIDCalc2(tx);

    double distance_adjust = (kpDistance * distance_error);

     if (camera == CAMERA.FRONT) {
      left_command += steering_adjust - distance_adjust;
      right_command -= steering_adjust + distance_adjust; 
    } else {
      left_command += steering_adjust + distance_adjust;
      right_command -= steering_adjust - distance_adjust;
    }
    Drivetrain.getInstance().tankDrive(left_command, right_command);

  }

 
  @Override
  protected boolean isFinished() {
    return finished;
  }

 
  @Override
  protected void end() {
  }


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

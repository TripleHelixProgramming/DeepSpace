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

public class driveByDocking extends Command {

  double Kp = 0.0305;
  double kpDistance = 0.0205;
  double min_command = 0.02;
  double left_command;
  double right_command;
  private boolean finished = false;

  private CAMERA camera;

  public driveByDocking(CAMERA camera) {

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

    if (Math.abs(tx) < 1) {
      finished = true;
    }

    if (tx > 0.0) {
      steering_adjust = Kp * tx - min_command;
    } else if (tx < 0.0) {
      steering_adjust = Kp * tx + min_command;
    }

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
}
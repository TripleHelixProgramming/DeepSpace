/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

public class JoshDrive extends NormalizedArcadeDrive {

  @Override
  public double throttleInputProcessing(double throttle) {
    return throttle;
  }

  @Override
  public double turnInputProcessing(double turn) {
    return turn;
  }
}

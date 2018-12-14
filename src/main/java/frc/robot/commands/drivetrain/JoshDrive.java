/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

public class JoshDrive extends JoystickDrive {
  @Override
  public double getThrottleScalar() {
    return 1;
  }

  @Override
  public double getTurnScalar() {
    return 1;
  }
}

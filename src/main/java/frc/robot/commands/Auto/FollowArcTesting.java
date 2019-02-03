/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.arcs.DistanceScalingArc;
import frc.arcs.TurnScalingArc;
import frc.robot.commands.FollowArc;
import frc.robot.commands.drivetrain.resetHeading;
import frc.robot.subsystems.Drivetrain;

public class FollowArcTesting extends CommandGroup {
  /**
   * Add your docs here.
   */
  public FollowArcTesting() {
  // Drivetrain.getInstance().resetHeading();
  addSequential(new resetHeading());
  addSequential(new FollowArc(Drivetrain.getInstance(), new TurnScalingArc(), false, false, false));
  addSequential(new resetHeading());
  addSequential(new FollowArc(Drivetrain.getInstance(), new TurnScalingArc(), true, true, false)); //to flip path the robot must also get flipped
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.robot_lifter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.RobotLifter;

public class ExtendLifter extends Command {
  public boolean isFinishedVal;
  public boolean isOperator;
  public double isMatchEnd;
  public ExtendLifter() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    //setupLogs();
    requires(RobotLifter.getInstance());
  
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    isOperator = DriverStation.getInstance().isOperatorControl();
    isMatchEnd = DriverStation.getInstance().getMatchTime();
    isFinishedVal = isFinished();
    if(isOperator == true && isMatchEnd <= 30.00){
        if(isFinishedVal == false){
          RobotLifter.getInstance().setPower(.65);
        }
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return RobotLifter.getInstance().isLimitSwitchTriggered();
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

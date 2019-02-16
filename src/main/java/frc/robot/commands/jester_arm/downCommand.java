/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.jester_arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ArmPreset;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.JesterArm;

public class downCommand extends Command {
  public downCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(JesterArm.getInstance());
    requires(CargoIntake.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    ArmPreset currentArmPreset = JesterArm.getInstance().getCurrentArmPreset();

    switch (currentArmPreset) {
    case FRONT_HATCH_LOWER:
      break;
    case FRONT_BALL_LOWER:
      JesterArm.getInstance().goTo(ArmPreset.FRONT_HATCH_LOWER);
      break;
    case FRONT_HATCH_MIDDLE:
      if (CargoIntake.getInstance().isDown()) {
        CargoIntake.getInstance().up();
        CargoIntake.getInstance().off();
      } else {
        JesterArm.getInstance().goTo(ArmPreset.FRONT_BALL_LOWER);
      }
      break;
    case FRONT_BALL_MIDDLE:
      JesterArm.getInstance().goTo(ArmPreset.FRONT_HATCH_MIDDLE);
      break;
    case FRONT_HATCH_UPPER:
      JesterArm.getInstance().goTo(ArmPreset.FRONT_BALL_MIDDLE);
      break;
    case FRONT_BALL_UPPER:
      JesterArm.getInstance().goTo(ArmPreset.FRONT_HATCH_UPPER);
      break;
    case BACK_HATCH_LOWER:
      break;
    case BACK_BALL_LOWER:
      JesterArm.getInstance().goTo(ArmPreset.BACK_HATCH_LOWER);
      break;
    case BACK_HATCH_MIDDLE:
      JesterArm.getInstance().goTo(ArmPreset.BACK_BALL_LOWER);
      break;
    case BACK_BALL_MIDDLE:
      JesterArm.getInstance().goTo(ArmPreset.BACK_HATCH_LOWER);
      break;
    case BACK_HATCH_UPPER:
      JesterArm.getInstance().goTo(ArmPreset.BACK_BALL_MIDDLE);
      break;
    case BACK_BALL_UPPER:
      JesterArm.getInstance().goTo(ArmPreset.BACK_HATCH_UPPER);
      break;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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

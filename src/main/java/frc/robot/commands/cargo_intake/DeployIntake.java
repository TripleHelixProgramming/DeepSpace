/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo_intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.JesterArm;
// import frc.robot.subsystems.JesterArm.ArmPos;

public class DeployIntake extends Command {

  public DeployIntake() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(CargoIntake.getInstance());
    // requires(JesterArm.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // ArmPos currentArmPos = JesterArm.getInstance().getCurrentArmPreset();

    // if (currentArmPos.getPos() < ArmPos.FRONT_HATCH_MIDDLE.getPos())
    //   JesterArm.getInstance().setArmMotionMagic(ArmPos.FRONT_HATCH_MIDDLE);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // ArmPos currentArmPos = JesterArm.getInstance().getCurrentArmPreset();
    // boolean armClear = (currentArmPos.getPos() >= ArmPos.FRONT_HATCH_MIDDLE.getPos());
    // if (armClear){
      CargoIntake.getInstance().down();
      CargoIntake.getInstance().in();
    // }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // JesterArm.getInstance().setArmMotionMagic(CalculateArmPos.FRONT_HATCH_LOWER);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

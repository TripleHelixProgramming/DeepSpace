/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.jester_wrist;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ArmPreset;
import frc.robot.OI;
import frc.robot.subsystems.JesterArm;
import frc.robot.subsystems.JesterWrist;

public class DriveWristByJoystick extends Command {

  private JesterWrist jesterWrist = JesterWrist.getInstance();

  int position;

  public DriveWristByJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super("Drive wrist by joystick");
    requires(jesterWrist);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    JesterArm.getInstance().setManualMode();
    position = jesterWrist.getWristPos();
    HelixEvents.getInstance().addEvent("JESTER WRIST", "Starting to drive wrist by joystick");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    int backLimit = ArmPreset.DELIVER_HATCH_LOWER.CalculateWristPos();
    int frontLimit = ArmPreset.PICKUP_HATCH.CalculateWristPos();

    position += OI.getInstance().getWristPower() * 10;
    if (position > backLimit) {
        position = backLimit;
    } else if (position < frontLimit) {
        position = frontLimit;
    }

    jesterWrist.setWristMotionMagic(position);

    SmartDashboard.putNumber("Arm Manual Position", position);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (JesterArm.getInstance().getCurrentArmPreset() != ArmPreset.MANUAL);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    HelixEvents.getInstance().addEvent("JESTER WRIST", "finished DriveWristByJoystick");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    HelixEvents.getInstance().addEvent("JESTER WRIST", "Interrupted DriveWristByJoystick");
  }
}
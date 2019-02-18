/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.hatch.GrabHatch;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.team2363.logger.HelixLogger;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class HatchGrabber extends Subsystem {
  private DigitalInput HatchLimit = new DigitalInput(RobotMap.HATCH_LIMIT_CHANNEL);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public DoubleSolenoid grabber = new DoubleSolenoid(RobotMap.HATCH_RELEASE, RobotMap.HATCH_GRAB);
  
  public static HatchGrabber INSTANCE = new HatchGrabber();
  
  public static HatchGrabber getInstance(){
    if (INSTANCE == null){
      INSTANCE = new HatchGrabber();
    }
    return INSTANCE;
  }

  public HatchGrabber () {
    // setupLogs();
  }
  public void hatchGrab(){
    grabber.set(DoubleSolenoid.Value.kForward);
  }
  public void hatchRelease(){
    grabber.set(DoubleSolenoid.Value.kReverse);
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // setDefaultCommand(new GrabHatch());
  }

  private void setupLogs() {
    HelixLogger.getInstance().addBooleanSource("HAS HATCH", HatchLimit::get);
  }
}

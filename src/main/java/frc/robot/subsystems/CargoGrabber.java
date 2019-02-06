/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.cargo_grabber.GrabCargo;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team2363.logger.HelixLogger;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Add your docs here.
 */
public class CargoGrabber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DoubleSolenoid cargo_solenoid = new DoubleSolenoid(RobotMap.CARGO_EXTEND_SOLENOID,
      RobotMap.CARGO_RETRACT_SOLENOID);

  private TalonSRX leftWheel = new TalonSRX(RobotMap.CARGO_LEFT_ID);
  private TalonSRX rightWheel = new TalonSRX(RobotMap.CARGO_RIGHT_ID);

  private static CargoGrabber INSTANCE = new CargoGrabber();

  public CargoGrabber() {
    setupLogs();
  }

  public static CargoGrabber getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new CargoGrabber();
    }
    return INSTANCE;
  }

  public void closeGrabber() {
    cargo_solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void openGrabber() {
    cargo_solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void intake() {
    leftWheel.set(ControlMode.PercentOutput, 0.35);
    rightWheel.set(ControlMode.PercentOutput, -0.35);
  }

  public void eject() {
    leftWheel.set(ControlMode.PercentOutput, -0.35);
    rightWheel.set(ControlMode.PercentOutput, 0.35);
  }

  public boolean isOpen() {
    if (cargo_solenoid.get() == DoubleSolenoid.Value.kReverse) {
      return true;
    }
    return false;
  }

  public double getOutputCurrent() {
    return Math.max(leftWheel.getOutputCurrent(), rightWheel.getOutputCurrent());
  }

  /**
   * Use to detect if the gear grabber roller is over current
   * 
   * @return true if over 20 amps
   */
  public boolean isOverCurrent() {
    if (getOutputCurrent() > 15) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new GrabCargo());
  }

  private void setupLogs() {
    HelixLogger.getInstance().addDoubleSource("LEFT_WHEEL_CURRENT", leftWheel::getOutputCurrent);
    HelixLogger.getInstance().addDoubleSource("RIGHT_WHEEL_CURRENT", rightWheel::getOutputCurrent);
    HelixLogger.getInstance().addBooleanSource("IS OPEN", CargoGrabber.getInstance()::isOpen);
  }
}

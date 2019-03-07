/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.RobotMap.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

// import frc.robot.commands.cargo_intake.RetractIntake;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CargoIntake extends Subsystem {

  public static CargoIntake INSTANCE = new CargoIntake();

  public static CargoIntake getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new CargoIntake();
    }
    return INSTANCE;
  }

  private VictorSPX motor = new VictorSPX(CARGO_INTAKE_ID);
  private DoubleSolenoid solenoid = new DoubleSolenoid(CARGO_INTAKE_DEPLOY, CARGO_INTAKE_RETRACT);
//CompBot
  // public void in() {
  //   motor.set(ControlMode.PercentOutput, -0.2);
  // }
//PracticeBot
  public void in() {
    motor.set(ControlMode.PercentOutput, -0.70);
  }

  public void out() {
    motor.set(ControlMode.PercentOutput, 0.4);
  }

  public void off() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  public void up() {
    solenoid.set(Value.kReverse);
  }

  public void down() {
    solenoid.set(Value.kForward);
  }

  public boolean isUp() {
    return solenoid.get() == Value.kReverse;
  }

  public boolean isDown() {
    return solenoid.get() == Value.kForward;
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // setDefaultCommand(new RetractIntake());
  }
}

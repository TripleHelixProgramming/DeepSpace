/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team2363.logger.HelixLogger;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class RobotLifter extends Subsystem {
  private TalonSRX left = new TalonSRX(RobotMap.ROBOT_LIFTER_LEFT);
  private TalonSRX right = new TalonSRX(RobotMap.ROBOT_LIFTER_RIGHT);

  private static RobotLifter INSTANCE = new RobotLifter();

  public static RobotLifter getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new RobotLifter();
    }
    return INSTANCE;
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public RobotLifter(){ 
    setupLogs();

    
    right.setNeutralMode(NeutralMode.Brake);
    right.configContinuousCurrentLimit(40, 0);
		right.configPeakCurrentLimit(60, 0);
		right.configPeakCurrentDuration(100, 0);
    right.enableCurrentLimit(true);
    
    left.setNeutralMode(NeutralMode.Brake);
    left.configContinuousCurrentLimit(40, 0);
		left.configPeakCurrentLimit(60, 0);
		left.configPeakCurrentDuration(100, 0);
    left.enableCurrentLimit(true);
  }
  public void setPower(double power){
    left.set(ControlMode.PercentOutput,power);
    right.set(ControlMode.PercentOutput, power);
  }

  private void setupLogs() {
    HelixLogger.getInstance().addDoubleSource("LEFT_MOTOR_CURRENT", left::getOutputCurrent);
    HelixLogger.getInstance().addDoubleSource("RIGHT_MOTOR_CURRENT", right::getOutputCurrent);
   }
}


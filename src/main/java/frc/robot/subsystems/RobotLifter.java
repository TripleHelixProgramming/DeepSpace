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
  private TalonSRX leftMaster = new TalonSRX(RobotMap.ROBOT_LIFTER_MASTER);
  private TalonSRX rightSlave = new TalonSRX(RobotMap.ROBOT_LIFTER_SLAVE);

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

    rightSlave.follow(leftMaster);
    rightSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.configContinuousCurrentLimit(40, 0);
		rightSlave.configPeakCurrentLimit(60, 0);
		rightSlave.configPeakCurrentDuration(100, 0);
    rightSlave.enableCurrentLimit(true);
    
    leftMaster.configContinuousCurrentLimit(40, 0);
		leftMaster.configPeakCurrentLimit(60, 0);
		leftMaster.configPeakCurrentDuration(100, 0);
    leftMaster.enableCurrentLimit(true);
  }
  public void setPower(double power){
    leftMaster.set(ControlMode.PercentOutput,power);
    rightSlave.set(ControlMode.PercentOutput, power);
  }

  private void setupLogs() {
    HelixLogger.getInstance().addDoubleSource("LEFT_MASTER_CURRENT", leftMaster::getOutputCurrent);
    HelixLogger.getInstance().addDoubleSource("RIGHT_SLAVE_CURRENT", rightSlave::getOutputCurrent);
   }
}


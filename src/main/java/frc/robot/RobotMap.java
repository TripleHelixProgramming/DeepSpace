/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  // PDP PORT NUMBERS
  public static int RIGHT_SLAVE_2_PDP = 0;       
  public static int RIGHT_SLAVE_1_PDP= 1;
  public static int RIGHT_MASTER_PDP = 2;

  public static int LIFTER_LEFT_PDP = 3;
  public static int WRIST_PDP = 4;
  public static int CARGO_INTAKE_PDP = 5;
  public static int ARM_MASTER_PDP = 6;
  public static int ARM_SLAVE_PDP = 7;   


  public static int CARGO_LEFT_PDP = 8;
  public static int CARGO_RIGHT_PDP = 9;
  public static int EMPTY_PDP = 10;
  public static int CAMERA_PDP = 11;
  public static int LIFTER_RIGHT_PDP = 12;

  public static int LEFT_MASTER_PDP = 13;
  public static int LEFT_SLAVE_1_PDP = 14;
  public static int LEFT_SLAVE_2_PDP = 15;

  //  CAN ID NUMBERS

  //  DRIVETRAIN
  public static int LEFT_MASTER_ID = 23;
  public static int LEFT_SLAVE_1_ID = 24;
  public static int LEFT_SLAVE_2_ID = 25;

  public static int RIGHT_MASTER_ID = 12;
  public static int RIGHT_SLAVE_1_ID = 11;
  public static int RIGHT_SLAVE_2_ID = 10;
  
  //  ARM
  public static int WRIST_ID = 14;
  public static int WRIST_ENCODER_ID = 4;
  public static int ARM_MASTER_ID = 16;
  public static int ARM_SLAVE_ID = 17;

  //  CargoGrabber 
  public static int CARGO_EXTEND_SOLENOID = 1;
  public static int CARGO_RETRACT_SOLENOID = 6;
  public static int CARGO_INTAKE_DEPLOY = 5;
  public static int CARGO_INTAKE_RETRACT = 2;
  public static int CARGO_LEFT_ID= 18;
  public static int CARGO_RIGHT_ID = 19;
  public static int CARGO_INTAKE_ID = 15;
  
  //  HATCHGRABBER
  public static int HATCH_GRAB = 7;
  public static int HATCH_RELEASE = 0;
  public static int HATCH_LIMIT_CHANNEL = 2;
  
  //  ROBOT LIFTER
  public static int LIFTER_LEFT_ID = 13;
  public static int LIFTER_RIGHT_ID = 22;
  
  //  OTHER
  public static int CTRE_TIMEOUT_INIT = 10;
  public static int CTRE_TIMEOUT_PERIODIC = 0;
}

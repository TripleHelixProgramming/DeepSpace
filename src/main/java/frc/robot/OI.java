/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.arcs.DistanceScalingArc;
import frc.arcs.SpeedTestingArc;
import frc.arcs.Straight10FeetArc;
import frc.arcs.TurnScalingArc;
import frc.robot.camera.CAMERA;
import frc.robot.commands.FollowArc;
import frc.robot.commands.Auto.FollowArcTesting;
import frc.robot.commands.Auto.MoveJesterTo;
import frc.robot.commands.drivetrain.driveByAssistJosh;
import frc.robot.commands.drivetrain.driveByCamera;
import frc.robot.commands.drivetrain.driveByDocking;
import frc.robot.commands.drivetrain.driveByDockingPID;
import frc.robot.commands.drivetrain.driveByVision;
import frc.robot.commands.jester_arm.MoveArmTo;
import frc.robot.commands.jester_arm.downCommand;
import frc.robot.commands.jester_arm.upCommand;


// import frc.robot.commands.drivetrain.driveByDockingPID;
// import frc.robot.commands.drivetrain.aimByVision;
// import frc.robot.commands.Camera.driveByVision;
// import frc.robot.commands.jester_arm.ToggleArmCommand;
// import frc.robot.commands.jester_arm.ToggleHeightCommand;
// import frc.robot.commands.Camera.driveByAssist;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private static OI INSTANCE;

  /**
   * @return retrieves the singleton instance of the Operator Interface
   */
  public static OI getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new OI();
    }
    return INSTANCE;
  }

  private final Joystick driver = new Joystick(ControllerMap.DRIVER_PORT);
  private final Joystick operator = new Joystick(ControllerMap.OPERATOR_PORT);

  private OI() {
    //Driver Controls

      new JoystickButton(driver, ControllerMap.X).whileHeld(new driveByDocking(CAMERA.FRONT));
      new JoystickButton(driver, ControllerMap.B).whileHeld(new driveByDocking(CAMERA.BACK));

      new JoystickButton(driver, ControllerMap.RB).whenPressed(new driveByCamera(CAMERA.FRONT));
      new JoystickButton(driver, ControllerMap.RB).whenPressed(new driveByCamera(CAMERA.BACK));

      new JoystickButton(driver, ControllerMap.Y).whileHeld(new driveByVision(CAMERA.FRONT));
      new JoystickButton(driver, ControllerMap.A).whileHeld(new driveByVision(CAMERA.BACK));

      // new JoystickButton(driver, ControllerMap.Y).whileHeld(new driveByDockingPID(CAMERA.FRONT));
      // new JoystickButton(driver, ControllerMap.A).whileHeld(new driveByDockingPID(CAMERA.BACK));

      new JoystickButton(driver, ControllerMap.LB).whileHeld(new FollowArcTesting());

      // new JoystickButton(operator, ControllerMap.B).whenPressed(new MoveArmTo(ArmPreset.FRONT_BALL_UPPER));
      // new JoystickButton(operator, ControllerMap.X).whenPressed(new MoveArmTo(ArmPreset.FRONT_HATCH_UPPER));
      // new JoystickButton(operator, ControllerMap.Y).whenPressed(new MoveArmTo(ArmPreset.BACK_HATCH_UPPER));

      // new JoystickButton(operator, ControllerMap.B).whenPressed(new MoveJesterTo(ArmPreset.FRONT_HATCH_UPPER));
      new JoystickButton(operator, ControllerMap.A).whenPressed(new downCommand());
      new JoystickButton(operator, ControllerMap.Y).whenPressed(new upCommand());
      // new JoystickButton(operator, ControllerMap.Y).whenPressed(new MoveArmTo(ArmPreset.FRONT_BALL_LOWER));
      // new JoystickButton(operator, ControllerMap.A).whenPressed(new MoveArmTo(ArmPreset.FRONT_HATCH_LOWER));

      // new JoystickButton(operator, ControllerMap.Y).whenPressed(new ToggleHeightCommand());

  }

  /**
   * @return the raw controller throttle
   */
  public double getThrottle() {
    return -driver.getRawAxis(ControllerMap.LEFT_STICK_Y); 
    // return -driver.getRawAxis(ControllerMap.LEFT_TRIGGER) + driver.getRawAxis(ControllerMap.RIGHT_TRIGGER);
	}
  
  /**
   * @return the raw controller turn
   */
  public double getTurn() {
    return driver.getRawAxis(ControllerMap.RIGHT_STICK_X);
  }
  
  public double getArmPower() {
    double stick = -operator.getRawAxis(ControllerMap.LEFT_STICK_Y);
    stick *= Math.abs(stick);
    if (Math.abs(stick) < 0.05) {
      stick = 0;
    }
    return stick;
  }

  public double getGMPOV() {
    return operator.getPOV();
  }
  
  /**
	 * Turns on and off the rumble function on the driver and operator controllers
	 * @param set true to turn on rumble
	 */
	public void setControllerRumble(boolean state) {
		if (state == true) {
			driver.setRumble(RumbleType.kLeftRumble, 1);
			driver.setRumble(RumbleType.kRightRumble, 1);  
			operator.setRumble(RumbleType.kLeftRumble, 1);
			operator.setRumble(RumbleType.kRightRumble, 1);
		} else {
			driver.setRumble(RumbleType.kLeftRumble, 0);
			driver.setRumble(RumbleType.kRightRumble, 0);
			operator.setRumble(RumbleType.kLeftRumble, 0);
			operator.setRumble(RumbleType.kRightRumble, 0);
		}
	}
}


/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.camera.CAMERA;

public class driveByAssist extends PIDCommand {

  private CAMERA camera;

  public driveByAssist(CAMERA camera) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    super(0.01, 0.001, 0.001); // P,I,D values to use on Aiming.
    requires(Drivetrain.getInstance());
    this.camera = camera;

    getPIDController().setInputRange(-27.0, 27.0);        // tx range of Limelight
    getPIDController().setOutputRange(-1.0, 1.0);         // Give us motor values between -1 & 1
    getPIDController().setAbsoluteTolerance(2);           // End when with in 2 degrees. See isFinished()

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Always set which camera you are using. This is passed into the command
    // from OI.java.

    setSetpoint(0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    camera.setDockingMode();
  }

  @Override
  protected double returnPIDInput() {
    // perfrom PID controller on X -- rotational degrees.
    return (camera.RotationalDegreesToTarget());
  }

  @Override
  protected void usePIDOutput(double output) {
    double throttleInput, saturatedInput, greaterInput, lesserInput, left, right;

    SmartDashboard.putNumber("PID Output: ", output);

    //  Do same thing Normalized Arcade Drive does to throttle and turn.
    throttleInput = OI.getInstance().getThrottle(); // forward throttle
    greaterInput = Math.max(Math.abs(throttleInput), Math.abs(output));
    // range [0, 1]
    lesserInput = Math.abs(throttleInput) + Math.abs(output) - greaterInput;
    // range [0, 1]
    if (greaterInput > 0.0) {
      saturatedInput = (lesserInput / greaterInput) + 1.0;
      // range [1, 2]
    } else {
      saturatedInput = 1.0;
    }

    // scale down the joystick input values
    // such that (throttle + turn) always has a range [-1, 1]
    throttleInput = throttleInput / saturatedInput;
    output = output / saturatedInput;
    
    if (camera == CAMERA.FRONT) {
      left = output - throttleInput;
      right = output+ throttleInput;
    } else {
      left = output + throttleInput;
      right = output - throttleInput;
    }

    // output is steering adjustment
    Drivetrain.getInstance().tankDrive(left, right);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return getPIDController().onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    getPIDController().disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

}

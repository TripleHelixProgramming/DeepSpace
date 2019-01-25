/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Camera;


import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Camera.CAMERA;

public class driveByAssist extends PIDCommand {

  private CAMERA location;

  public driveByAssist(CAMERA location) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super(0.01, 0.001, 0.001);   // PID values to use on Aiming.

    this.location = location;

    requires(Drivetrain.getInstance());
    requires(Camera.getInstance());

    getPIDController().setAbsoluteTolerance(2);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

      // Always set which camera you are using. This is passed into the command 
      // from OI.java.
      Camera.getInstance().setCamera(location);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  @Override
  protected double returnPIDInput() {
    // perfrom PID controller on X -- rotational degrees.
    return (Camera.getInstance().RotationalDegreesToTarget());
  }

  @Override
  protected void usePIDOutput(double output) {
    SmartDashboard.putNumber("PID Output: ", output);

    // Output is the PID out tx -- rotational degrees
    // Forward is just driver Y stick value.
    
    double distance_adjust = OI.getInstance().getDriverY();

    Drivetrain.getInstance().tankDrive(output - distance_adjust, output + distance_adjust);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

}

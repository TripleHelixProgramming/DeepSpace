/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.camera.CAMERA;

public class driveByVision extends Command {

  private static final double Kf = 0.02;
  private static final double Kp = 0.0305;
  private static final double Ki = 0.00;
  private static final double Kd = 0.00;

  // 2 PID Controllers - one for distance (ty) and one for aiming (tx)
  private PIDController distancePID;
  private PIDController aimPID;

  DTOutput distancePIDresult = new DTOutput();
  DTOutput aimPIDresult = new DTOutput();

  private CAMERA camera;    //  Which camera are we using -- equated to robot orientation

  private class DTOutput implements PIDOutput {
    double output;

    public DTOutput() {
      output = 0;
    }

    @Override
    public void pidWrite (double output) {
      this.output = output;
    }
    public double get() {
      return output;
    }
  }

  public driveByVision(CAMERA camera) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Drivetrain.getInstance());
    this.camera = camera;

    //
    // Define PIDSource based on distance to travel - ty
    //
    PIDSource distanceSource = new PIDSource() {
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
      }

      @Override
      public PIDSourceType getPIDSourceType() {
        // Distance type PID
        return PIDSourceType.kDisplacement;
      }

      @Override
      public double pidGet() {
        return camera.VerticalDegreesToTarget();
      }
    };

    //
    // Define PIDSource based on aim -- tx
    //
    PIDSource aimSource = new PIDSource() {
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
      }

      @Override
      public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
      }

      @Override
      public double pidGet() {
        return camera.RotationalDegreesToTarget();
      }
    };

    distancePID = new PIDController(Kp, Ki, Kd, Kf, distanceSource, distancePIDresult);
    distancePID.setInputRange(-20.5, 20.5);   // ty range of LimeLight
    distancePID.setOutputRange(-1.0, 1.0);    // Give us motor values between -1 & 1
    distancePID.setAbsoluteTolerance(2);      // End when with in 2 degrees. See isFinished()

    aimPID = new PIDController(Kp, Ki, Kd, Kf, aimSource, aimPIDresult);
    aimPID.setInputRange(-27.0, 27.0);        // tx range of Limelight
    aimPID.setOutputRange(-1.0, 1.0);         // Give us motor values between -1 & 1
    aimPID.setAbsoluteTolerance(2);           // End when with in 2 degrees. See isFinished()
  
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    camera.setDockingMode();

    distancePID.setSetpoint(0.0);
    aimPID.setSetpoint(0.0);

    distancePID.enable();
    aimPID.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double saturatedInput;
    double left_command, right_command = 0.0; 

    double throttleInput = distancePIDresult.get();
    double turnInput = aimPIDresult.get();

    //  DO NATE's Normalization stuff on throttle & turn values.
    //=============================================================
    double greaterInput = Math.max(Math.abs(throttleInput), Math.abs(turnInput));
      //range [0, 1]
    double lesserInput = Math.abs(throttleInput) + Math.abs(turnInput) - greaterInput;
      //range [0, 1]

    if (greaterInput > 0.0) {
      saturatedInput = (lesserInput / greaterInput) + 1.0;
         //range [1, 2]
    } else {
      saturatedInput = 1.0;
    }
     
    //scale down the joystick input values
    //such that (throttle + turn) always has a range [-1, 1]
    throttleInput = throttleInput / saturatedInput;
    turnInput = turnInput / saturatedInput;

    //  NOW that throttle is Normalized....

    if (camera == CAMERA.FRONT) {
      left_command = turnInput - throttleInput;
      right_command = turnInput + throttleInput;
    } else {
      left_command = turnInput + throttleInput;
      right_command = turnInput - throttleInput;
    }

    Drivetrain.getInstance().tankDrive(left_command, right_command);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return distancePID.onTarget() && aimPID.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    distancePID.disable();
    aimPID.disable();
    Drivetrain.getInstance().tankDrive(0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

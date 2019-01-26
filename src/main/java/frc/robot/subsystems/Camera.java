/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.Camera.driveByCamera;

/**
 * Add your docs here.
 */
public class Camera extends Subsystem {

  public enum CAMERA {
    FRONT, BACK;
  }

  private static Camera INSTANCE = new Camera();

  private static NetworkTable frontCamera = getCamera(CAMERA.FRONT);
  private static NetworkTable backCamera = getCamera(CAMERA.BACK);
  public static NetworkTable currentCamera;

  /**
   * @return the singleton instance of the LimeLight subsystem
   */
  public static Camera getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Camera();
    }
    return INSTANCE;
  }

  private Camera() {
    currentCamera = NetworkTableInstance.getDefault().getTable("Limelight-front");
    frontCamera = NetworkTableInstance.getDefault().getTable("limelight-front");
    backCamera = NetworkTableInstance.getDefault().getTable("limelight-back");
  }

  public static NetworkTable getCamera(CAMERA location) {
    if (location == CAMERA.FRONT) {
      // if (currentCamera == null) {
        currentCamera = frontCamera;
      
        return currentCamera;
    } else {
     
        currentCamera = backCamera;
      
       return currentCamera;
    }
  }

  public void setCamera(CAMERA location) {
    currentCamera = getCamera(location);
  }

  public void setCameraMode() {
    currentCamera.getEntry("camMode").setNumber(1);
    // backCamera.getEntry("camMode").setNumber(1);

    currentCamera.getEntry("ledMode").setNumber(1);
    // backCamera.getEntry("ledMode").setNumber(1);

  }

  public void setDockingMode() { 
      currentCamera.getEntry("camMode").setNumber(0);
      currentCamera.getEntry("ledMode").setNumber(0);
      currentCamera.getEntry("pipeline").setNumber(0);
      currentCamera.getEntry("stream").setNumber(0);

  }

  // public void setVisionMode() {
  //     currentCamera.getEntry("camMode").setNumber(0);
  //     currentCamera.getEntry("ledMode").setNumber(0);
  //     currentCamera.getEntry("pipeline").setNumber(0);
  //     currentCamera.getEntry("stream").setNumber(0);
  // }

  public boolean IsTargetFound() {
    double v = currentCamera.getEntry("tx").getDouble(0.0);
    return ((v == 0.0)? false: true);
  }

  public double RotationalDegreesToTarget() {
    return currentCamera.getEntry("tx").getDouble(0.0);
  }

  public double VerticalDegreesToTarget() {
    return currentCamera.getEntry("ty").getDouble(0.0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new driveByCamera(CAMERA.FRONT));
    setDefaultCommand(new driveByCamera(CAMERA.BACK));
  }
}
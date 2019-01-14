/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
// import sun.security.jca.GetInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Camera.driveByCamera;

/**
 * Add your docs here.
 */
public class Camera extends Subsystem {

  enum CAMERA {
    FRONT, BACK;
  }
  
  private static Camera INSTANCE = new Camera();
  private static NetworkTable frontCamera = getCamera(CAMERA.FRONT);
  private static NetworkTable backCamera = getCamera(CAMERA.BACK);

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
 
  }


  private static NetworkTable getCamera(CAMERA Camera) {
    if (Camera == CAMERA.FRONT) {
      if (frontCamera == null) {
        frontCamera = NetworkTableInstance.getDefault().getTable("front"); 
      }
      return frontCamera;
    } else { 
      if (backCamera == null) {
        backCamera = NetworkTableInstance.getDefault().getTable("back");
      }
      return backCamera;
    }
  } 

  public void setCameraMode() {
    frontCamera.getEntry("camMode").setNumber(1);
    backCamera.getEntry("camMode").setNumber(1);
  }

  public void setDockingMode() {
    frontCamera.getEntry("camMode").setNumber(0);
    backCamera.getEntry("camMode").setNumber(0);
  }

  public boolean getIsTargetFoundFront() {
    NetworkTableEntry tv = frontCamera.getEntry("tv");
    double v = tv.getDouble(0);
    if (v == 0.0f){
        return false;
    }else {
        return true;
    }
}

public boolean getIsTargetFoundBack() {
  NetworkTableEntry tv = backCamera.getEntry("tv");
  double v = tv.getDouble(0);
  if (v == 0.0f){
      return false;
  }else {
      return true;
  }
}

public double getdegRotationToTargetFront() {
  NetworkTableEntry txFront = frontCamera.getEntry("tx");
  double x = txFront.getDouble(0.0);
  return x;
}

public double getdegRotationToTargetBack() {
  NetworkTableEntry txBack = backCamera.getEntry("tx");
  double x = txBack.getDouble(0.0);
  return x;
}

public double getdegVerticalToTargetFront() {
  NetworkTableEntry tyFront = frontCamera.getEntry("ty");
  double y = tyFront.getDouble(0.0);
  return y;
}

public double getdegVerticalToTargetBack() {
  NetworkTableEntry tyBack = backCamera.getEntry("ty");
  double y = tyBack.getDouble(0.0);
  return y;
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new driveByCamera());
  }
}

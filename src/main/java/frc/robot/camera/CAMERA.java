/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.camera;

import edu.wpi.first.networktables.NetworkTableInstance;

public enum CAMERA {
    FRONT("limelight-front"), BACK("limelight-back");

    public final String name;

    CAMERA(String name) {
        this.name = name;
    }

    public void setCameraMode() {
        NetworkTableInstance.getDefault().getTable(CAMERA.FRONT.name()).getEntry("camMode").getDouble(1);
        NetworkTableInstance.getDefault().getTable(CAMERA.FRONT.name()).getEntry("ledMode").setNumber(1);
        NetworkTableInstance.getDefault().getTable(CAMERA.BACK.name()).getEntry("camMode").getDouble(1);
        NetworkTableInstance.getDefault().getTable(CAMERA.BACK.name()).getEntry("ledMode").setNumber(1);

    }

    public void setDockingMode() {
        NetworkTableInstance.getDefault().getTable(name).getEntry("camMode").getDouble(0);
        NetworkTableInstance.getDefault().getTable(name).getEntry("ledMode").setNumber(0);
        NetworkTableInstance.getDefault().getTable(name).getEntry("pipeline").setNumber(0);
        NetworkTableInstance.getDefault().getTable(name).getEntry("stream").setNumber(0);

    }

    public boolean IsTargetFound() {
        double v = NetworkTableInstance.getDefault().getTable(name).getEntry("tx").getDouble(0.0);
        return ((v == 0.0) ? false : true);
    }

    public double RotationalDegreesToTarget() {
        return NetworkTableInstance.getDefault().getTable(name).getEntry("tx").getDouble(0.0);
    }

    public double VerticalDegreesToTarget() {
        return NetworkTableInstance.getDefault().getTable(name).getEntry("ty").getDouble(0);
    }
}

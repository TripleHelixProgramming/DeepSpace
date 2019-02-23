/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public enum ArmPreset {

    // shoulder angle, wrist angle
    PICKUP_HATCH(-70, 0), 
    CARGO_WP(0, -60), 
    PICKUP_CARGO_FLOOR(-20, -90), 
    CARGO(0, 0), 
    DELIVER_BALL_UPPER(110, 135),
    DELIVER_HATCH_UPPER(110, 175), 
    DELIVER_BALL_MIDDLE(153, 180), 
    DELIVER_HATCH_MIDDLE(173, 180), 
    DELIVER_BALL_LOWER(220, 180),
    DELIVER_HATCH_LOWER(250, 180),
    STOW(250, 180),
    UNPACK_WP(226, 284), 
    START(238, 284);
    
    private final int shoulder_angle;
    private final int wrist_angle;

    // Practice Bot slope & y-intercept for arm & wrist to calculate sensor positions
    // (requires linear sensor)
    public double m_wrist = -0.489;
    public double b_wrist = 246;
    public double m_shoulder = 0.859;
    public double b_shoulder = -499;

    // Comp Bot slope & y-intercept for arm & wrist to calculate sensor positions
    // (requires linear sensor)
    // public double m_wrist = -0.494;
    // public double b_wrist = 560;
    // public double m_shoulder = 0.861;
    // public double b_shoulder = -255;


    private ArmPreset(int shoulder_angle, int wrist_angle) {
        this.shoulder_angle = shoulder_angle;
        this.wrist_angle = wrist_angle;
    }

    public int getWristAngle() {
        return wrist_angle;
    }

    public int getShoulderAngle() {
        return shoulder_angle;
    }

    public int CalculateArmPos() {
        return (int) (m_shoulder * shoulder_angle + b_shoulder);
    }

    public int CalculateWristPos() {
        return (int) (m_wrist * wrist_angle + b_wrist);
    }

    public int getShoulderAngle(int armPos) {
        return (int) ((armPos + b_shoulder) / m_shoulder);
    }

    public int getWristLowerLimit(int curArmPos) {
        int wristLowerLimit;

        wristLowerLimit = Math.min(280, curArmPos + 70);
        wristLowerLimit = (int) (m_wrist * wristLowerLimit + b_wrist);

        return wristLowerLimit;

    }

    public int getWristUpperLimit(int curArmPos) {
        int wristUpperLimit;

        wristUpperLimit = Math.max(-100, curArmPos - 70);
        wristUpperLimit = (int) (m_wrist * wristUpperLimit + b_wrist);

        return wristUpperLimit;
    }

}

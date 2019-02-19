/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public enum ArmPreset {

    // shoulder angle, wrist angle, next_fwd index, next_aft index, mirror index
    FRONT_HATCH_LOWER(-70, 0), 
    FRONT_BALL_LOWER(-50, 0), 
    FRONT_HATCH_MIDDLE(0, 0), 
    FRONT_BALL_MIDDLE(20, 0),
    FRONT_HATCH_UPPER(70, 30), 
    FRONT_BALL_UPPER(80, 30), 
    TRANSITION(90, 90), 
    BACK_BALL_UPPER(100, 150),
    BACK_HATCH_UPPER(110, 150), 
    BACK_BALL_MIDDLE(160, 180), 
    BACK_HATCH_MIDDLE(180, 180), 
    BACK_BALL_LOWER(230, 180),
    BACK_HATCH_LOWER(250, 180), 
    START(230, 280), 
    UNPACK_WP(210, 180), 
    PICK_UP(250, 200);

    private final int shoulder_angle;
    private final int wrist_angle;

    // Practice Bot slope & y-intercept for arm & wrist to calculate sensor positions
    // (requires linear sensor)
    // public double m_wrist = -0.489;
    // public double b_wrist = 246;
    // public double m_shoulder = 0.859;
    // public double b_shoulder = -499;

    // Comp Bot slope & y-intercept for arm & wrist to calculate sensor positions
    // (requires linear sensor)
    public double m_wrist = -0.494;
    public double b_wrist = 560;
    public double m_shoulder = 0.859;
    public double b_shoulder = -260;

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

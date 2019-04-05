/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public enum ArmPreset {

    //shoulder angle, wrist angle CompBot
    PICKUP_HATCH(-72, -2), 
    CARGO_WP(0, -60), 
    MANUAL(0,0),

    //PICKUP_CARGO_FLOOR(-20, -97), //was 29,94
    PICKUP_CARGO_FLOOR_BACK(-55, -48),
    PICKUP_CARGO_FLOOR_FRONT(239, 217),

    CARGO_TRANSITION_WP(-15, -50),

    // CARGO(-30, 30), 
    CARGO(28, -31),
    DELIVER_SIDE (80, 120),
    DELIVER_BALL_UPPER(110, 139),
    DELIVER_HATCH_UPPER(110, 135), 
    BALL_TRANSITION_UPPER(123, 190),
    // DELIVER_BALL_MIDDLE(143, 182),      //original value
    // DELIVER_BALL_MIDDLE(193, 130),        //possible testing value
    DELIVER_BALL_MIDDLE(140, 188),        //possible testing value
    BALL_TRANSITION_LOWER(163, 150),
    DELIVER_HATCH_MIDDLE(168, 180), 
    // DELIVER_BALL_LOWER(200, 180),    //original
    DELIVER_BALL_LOWER(230, 160),    //latest possible testing angle
    // DELIVER_BALL_LOWER(184, 203),    //possible testing angle
    DELIVER_HATCH_LOWER(241, 180),
    STOW(250, 180),
    UNPACK_WP3(210, 270), 
    // UNPACK_WP1(225, 294),    //practive bot
    UNPACK_WP1(222, 298),   //comp bot
    // START(235, 292); //practice bot
    START(236, 314);    //comp bot


     // shoulder angle, wrist angle PracticeBot
    //  PICKUP_HATCH(-72, -2), 
    //  CARGO_WP(0, -60), 
    //  PICKUP_CARGO_FLOOR(-20, -90), 
    //  CARGO(0, 0), 
    //  DELIVER_BALL_UPPER(110, 135),
    //  DELIVER_HATCH_UPPER(110, 175), 
    //  DELIVER_BALL_MIDDLE(153, 180), 
    //  DELIVER_HATCH_MIDDLE(173, 180), 
    //  DELIVER_BALL_LOWER(220, 180),
    //  DELIVER_HATCH_LOWER(252, 180),
    //  STOW(250, 180),
    //  UNPACK_WP3(210, 270), 
    //  UNPACK_WP1(225, 294),
    //  UNPACK_WP2(221,271),
    //  START(235, 292);

    private final int shoulder_angle;
    private final int wrist_angle;

    // Practice Bot slope & y-intercept for arm & wrist to calculate sensor positions
    // (requires linear sensor)
    public double m_wrist = -0.472;
    public double b_wrist = 286;
    public double m_shoulder = 0.833;
    public double b_shoulder = -499;

    // Comp Bot slope & y-intercept for arm & wrist to calculate sensor positions
    // (requires linear sensor)
    // public double m_wrist = -0.483;
    // public double b_wrist = 549;
    // public double m_shoulder = 0.839;
    // public double b_shoulder = -248;

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

    //  Calculate Arm angle given arm sensor position
    public int CalcArmAngle(int armPos) {
        return (int) ((armPos - b_shoulder)/ m_shoulder);
    }

    //Caclulate Wrist Angle given wrist sensor position
    public int CalcWristAngle(int wristPos) {
        return (int) ((wristPos - b_wrist)/ m_wrist);
    }

    public int WristAngleToPos(int angle) {
        return (int) (m_wrist * angle + b_wrist);
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

    public int getWristLowerLimit(int curArmAngle) {
        int wristLowerLimit;

        wristLowerLimit = Math.min(280, curArmAngle + 70);
        wristLowerLimit = (int) (m_wrist * wristLowerLimit + b_wrist);

        return wristLowerLimit;

    }

    public int getWristUpperLimit(int curArmAngle) {
        int wristUpperLimit;

        wristUpperLimit = Math.max(-100, curArmAngle - 70);
        wristUpperLimit = (int) (m_wrist * wristUpperLimit + b_wrist);

        return wristUpperLimit;
    }

}

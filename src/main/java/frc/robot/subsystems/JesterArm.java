/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team2363.logger.HelixLogger;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.jester_arm.DriveArmByJoystick;
import frc.robot.commands.jester_arm.StopArm;

public class JesterArm extends Subsystem {

    private TalonSRX armMaster = new TalonSRX(RobotMap.ARM_MASTER_ID);
    private VictorSPX armSlave = new VictorSPX(RobotMap.ARM_SLAVE_ID);

    public static int ARM_ACCELERATION = 10;
    public static int ARM_CRUISE = 20;

    public enum ArmPos {
        START(200),
        FRONT_HATCH_LOWER(ArmPos.START.pos+100),
        FRONT_BALL_LOWER(ArmPos.START.pos+150),
        FRONT_HATCH_MIDDLE(ArmPos.START.pos+200),
        FRONT_BALL_MIDDLE(ArmPos.START.pos+250),
        FRONT_HATCH_UPPER(ArmPos.START.pos+300),
        FRONT_BALL_UPPER(ArmPos.START.pos+350),
        BACK_HATCH_UPPER(ArmPos.START.pos+400),
        BACK_BALL_UPPER(ArmPos.START.pos+450),
        BACK_HATCH_MIDDLE(ArmPos.START.pos+500),
        BACK_BALL_MIDDLE(ArmPos.START.pos+550),
        BACK_HATCH_LOWER(ArmPos.START.pos+600),
        BACK_BALL_LOWER(ArmPos.START.pos+630),
        BACK_LIMIT(ArmPos.START.pos+650);

        private final int pos;

        private ArmPos(int pos) {
            this.pos = pos;
        }

        public int getPos() {
            return pos;
        }
    }

    private static JesterArm INSTANCE = new JesterArm();
    private ArmPos currentArmPreset = ArmPos.START;
    PowerDistributionPanel pdp = new PowerDistributionPanel();

    /**
     * @return the singleton instance of the JesterArm subsystem
     */
    public static JesterArm getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new JesterArm();
        }
        return INSTANCE;
    }

    public JesterArm() {
        super("Jester Arm Subsystem");
        setupLogs();

        currentArmPreset = ArmPos.START;

		armSlave.follow(armMaster);
		armSlave.setNeutralMode(NeutralMode.Brake);
        armSlave.configOpenloopRamp(0.2, 0);

        armMaster.setNeutralMode(NeutralMode.Brake);
        armMaster.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.config_kP(0, 1, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.config_kI(0, 0.01, RobotMap.CTRE_TIMEOUT_INIT);


        // Set current limiting
        armMaster.configContinuousCurrentLimit(40, 0);
        armMaster.configPeakCurrentLimit(60, 0);
        armMaster.configPeakCurrentDuration(100, 0);
        armMaster.enableCurrentLimit(true);

        // The arm starts the match in a one-time docked position.  Move arm from
        // docked position to front lower scoring position.

        // setArmSoftLimits(ArmPos.START.pos, ArmPos.BACK_LIMIT.pos);
        // setArmMotionProfile(ARM_ACCELERATION, ARM_CRUISE);
        // unDockArm();
    }

    private void setupLogs() {
        HelixLogger.getInstance().addDoubleSource("ARM MASTER CURRENT", armMaster::getOutputCurrent);
        HelixLogger.getInstance().addDoubleSource("ARM SLAVE", () -> pdp.getCurrent(RobotMap.ARM_SLAVE_ID));
    }

    public void setArmHeight(int pos) {
        armMaster.set(ControlMode.Position, pos);
    }
    
    // Move arm from docked position (at start of match) to front lower scoring position.
    public void unDockArm() {
        setArmMotionMagic(ArmPos.FRONT_HATCH_LOWER);
    }

    public void setArmMotionMagic(ArmPos preset) {
        setArmMotionMagic(preset.getPos());
        currentArmPreset = preset;
    }

    public void setArmMotionMagic(int pos) {
        // armMaster.set(ControlMode.MotionMagic, pos);
    }

    public void goTo(ArmPos pos) {
        goTo(pos.getPos());
    }

    public void goTo(int pos) {
        setArmMotionMagic(pos);
    }

    public int getArmPos() {
        return armMaster.getSelectedSensorPosition(0);
    }

    public ArmPos getCurrentArmPreset() {
        return currentArmPreset;         
    }

    public void setArmSoftLimits(int reverseSoftLimit, int forwardSoftLimit) {
        armMaster.configReverseSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT_PERIODIC);
        armMaster.configReverseSoftLimitThreshold(reverseSoftLimit, RobotMap.CTRE_TIMEOUT_PERIODIC);

        armMaster.configForwardSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT_PERIODIC);
        armMaster.configForwardSoftLimitThreshold(forwardSoftLimit, RobotMap.CTRE_TIMEOUT_PERIODIC);
    }

    public void setArmMotionProfile(int acceleration, int cruise) {
        armMaster.configMotionAcceleration(acceleration, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.configMotionCruiseVelocity(cruise, RobotMap.CTRE_TIMEOUT_INIT);
    }

    public void stop() {
        armMaster.set(ControlMode.PercentOutput, 0.0);
    }
    
    // Put items in here that you want updated on SmartDash during disableperiodic()
    public void updateSmartDash() {
        SmartDashboard.putNumber("Arm Pos", getArmPos());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Pos", getArmPos());
    }

    @Override
    protected void initDefaultCommand() {
        // setDefaultCommand(new StopArm());
    }
}

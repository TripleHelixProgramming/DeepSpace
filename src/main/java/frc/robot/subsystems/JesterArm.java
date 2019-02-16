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
import frc.robot.ArmPreset;

public class JesterArm extends Subsystem {

    private TalonSRX armMaster = new TalonSRX(RobotMap.ARM_MASTER_ID);
    private VictorSPX armSlave = new VictorSPX(RobotMap.ARM_SLAVE_ID);

    public static int ARM_ACCELERATION = 30;
    public static int ARM_CRUISE = 4;

    private static JesterArm INSTANCE = new JesterArm();
    private ArmPreset currentArmPreset = ArmPreset.BACK_HATCH_UPPER;

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
        // setupLogs();

        armSlave.configFactoryDefault();
        armMaster.configFactoryDefault();

        currentArmPreset = ArmPreset.FRONT_HATCH_LOWER;

		armSlave.follow(armMaster);
		armSlave.setNeutralMode(NeutralMode.Brake);
        armSlave.configOpenloopRamp(0.2, 0);

        armMaster.setNeutralMode(NeutralMode.Brake);
        armMaster.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.configFeedbackNotContinuous(true, RobotMap.CTRE_TIMEOUT_INIT);

        // Need to verify and set.  With positive motor direction sensor values should increase.
        armMaster.setSensorPhase(true);   
        armMaster.setInverted(false); 

        // PID Settings
        armMaster.config_kF(0, 0, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.config_kP(0, 1, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.config_kI(0, 0.01, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.config_kD(0, 0, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.configAllowableClosedloopError(0, 2, RobotMap.CTRE_TIMEOUT_INIT);

        // Set current limiting
        armMaster.configContinuousCurrentLimit(40, 0);
        armMaster.configPeakCurrentLimit(60, 0);
        armMaster.configPeakCurrentDuration(100, 0);
        armMaster.enableCurrentLimit(true);

        int reverseSoftLimit = ArmPreset.FRONT_BALL_MIDDLE.CalculateArmPos();
        int fwdSoftLimit = ArmPreset.BACK_BALL_MIDDLE.CalculateArmPos();
        setArmSoftLimits(reverseSoftLimit, fwdSoftLimit);
        setArmMotionProfile(ARM_ACCELERATION, ARM_CRUISE);

        // The arm starts the match in a one-time docked position.  Move arm from
        // docked position to front lower scoring position.
        unDockArm();
    }

    private void setupLogs() {
        HelixLogger.getInstance().addDoubleSource("ARM MASTER CURRENT", armMaster::getOutputCurrent);
        HelixLogger.getInstance().addDoubleSource("ARM SLAVE", () -> pdp.getCurrent(RobotMap.ARM_SLAVE_ID));
    }
    
    // Move arm from docked position (at start of match) to front lower scoring position.
    public void unDockArm() {
        // setArmMotionMagic(ArmPos.FRONT_HATCH_LOWER);
    }

    public void goTo(ArmPreset preset) {
        int newPos = preset.CalculateArmPos();
        currentArmPreset = preset;

        SmartDashboard.putNumber("Calc Arm Pos", newPos);
        setArmMotionMagic(newPos);
    }

    public void goTo(int pos) {
        setArmMotionMagic(pos);
    }

    public void setArmMotionMagic(int pos) {
        armMaster.set(ControlMode.MotionMagic, pos);
    }

    public int getArmPos() {
        return armMaster.getSelectedSensorPosition(0);
    }

    public ArmPreset getCurrentArmPreset() {
        return currentArmPreset;         
    }

    public void setArmSoftLimits(int reverseSoftLimit, int forwardSoftLimit) {

        SmartDashboard.putNumber("Arm Lower Limit", reverseSoftLimit);
        armMaster.configReverseSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT_PERIODIC);
        armMaster.configReverseSoftLimitThreshold(reverseSoftLimit, RobotMap.CTRE_TIMEOUT_PERIODIC);

        SmartDashboard.putNumber("Arm Upper Limit", forwardSoftLimit);
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
        updateSmartDash(); 
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new StopArm());
    }
}

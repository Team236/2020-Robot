/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {

  private CANSparkMax master;
  private CANDigitalInput bottomLimit;

  private DigitalInput newLimit;

  private Relay lockRelay;

  /**
   * Creates a new Climber.
   */
  public Climber() {

    master = new CANSparkMax(ID_MASTER, MotorType.kBrushless);

    lockRelay = new Relay(RELAY_PORT);

    bottomLimit = master.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    // newLimit = new DigitalInput(2);
  }

  private void setSpeedRaw(double speed) {
    master.set(speed);
  }

  public void stop() {
    setSpeedRaw(0);
  }

  public boolean isBottomLimit() {
    bottomLimit.enableLimitSwitch(false);
    return bottomLimit.get();
  }

  public boolean isNewLimit() {
    // return !newLimit.get();
    return false;
  }

  /* public double getEncoder() {
    return master.getSelectedSensorPosition();
  }

  public void resetEncoder() {
    master.setSelectedSensorPosition(0);
  } */

  /**
   * Sets pre-determined speed, takes into account limits
   */
  public void setSpeed(double speed) {
     if (isBottomLimit() && speed < 0) {
      stop();
    } else {
      setSpeedRaw(speed);
    } 
    // setSpeedRaw(speed);
  }

  /**
   * Sets to specified speed unless past encoder limit
   */
 /*  public void setSpeedEnc() {
    if (getEncoder() < ENC_LIMIT) {
      setSpeed(SPEED);
    } else {
      setSpeed(0.2);
    }
  } */

  /**
   * Powers relay: LL on, climb lock off
   */
  public void relayOn() {
    lockRelay.set(Value.kForward);
  }

  /**
   * Cuts power to relay: LL off, climb lock engages
   */
  public void relayOff() {
    lockRelay.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("climb limit", isBottomLimit());
    SmartDashboard.putBoolean("new limit", isNewLimit());
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  public CANSparkMax turretSpinner;
  // private Counter counter;
  private DigitalInput limitLeft, limitRight;
  // private CANEncoder encoder;
  public boolean wasHitLeft;
  public boolean wasHitRight;
  private boolean areLimitsUnplugged = false;

  public Turret() {
    turretSpinner = new CANSparkMax(Constants.TurretConstants.ID_TURRET, MotorType.kBrushless);
    // counter = new Counter(Constants.TurretConstants.DIO_TURRET);

    // Limit switches
    try {
      limitLeft = new DigitalInput(Constants.TurretConstants.DIO_LEFT_LIMIT);
      limitRight = new DigitalInput(Constants.TurretConstants.DIO_RIGHT_LIMIT);
    } catch (Exception e) {
      areLimitsUnplugged = true;
    }

    // turretSpinner.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
    // LimitSwitchNormal.NormallyOpen, 0);
    // turretSpinner.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
    // LimitSwitchNormal.NormallyOpen, 1);
    // limitLeft =
    // turretSpinner.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    // limitRight =
    // turretSpinner.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

  }

  private void setTurretSpeedRaw(double speed) {
    turretSpinner.set(speed);
  }

  /*
   * public void resetEncoders() { counter.reset(); encoder.setPosition(0); }
   * 
   * public double getEncoder() { return encoder.getPosition(); }
   */
  public double getRawSpeed() {
    return turretSpinner.getEncoder().getVelocity();
  }

  public boolean isLeftLimit() {
    // limitLeft.enableLimitSwitch(false);
    if (areLimitsUnplugged) {
      return false;
    } else {
      return limitLeft.get();

    }
  }

  public boolean isRightLimit() {
    // limitRight.enableLimitSwitch(false);
    if (areLimitsUnplugged) {
      return false;
    } else {
      return limitRight.get();
    }
    // limitLeft.enableLimitSwitch(false);
    // return limitLeft.get();
  }

  /*
   * public boolean isRightLimit() { //limitRight.enableLimitSwitch(false); return
   * limitRight.get(); }
   */

  /**
   * Sets speed considering limits
   * 
   * @param speed
   */
  public void setSpeed(double speed) {
    if (!isLeftLimit() && !isRightLimit()) {
      setTurretSpeedRaw(speed);
    } else {
      stop();
    }
  }

  public void setWasHitRight(boolean bool) {
    wasHitRight = bool;
  }

  public void setWasHitLeft(boolean bool) {
    wasHitLeft = bool;
  }

  public boolean getWasHitRight() {
    return wasHitRight;
  }

  public boolean getWasHitLeft() {
    return wasHitLeft;
  }

  /**
   * sets speed considering limits, previous conditions
   * 
   * @param speed
   * @param spinCase direction
   */
  public void set(double speed, int spinCase) {
    if (isLeftLimit() == false) {
      setWasHitLeft(true);
    }
    if (isRightLimit() == false) {
      setWasHitRight(true);
    }

    if (spinCase == 1 && (getWasHitRight() == false || isRightLimit() != false)) {
      setTurretSpeedRaw(-speed);
    } else if (getWasHitRight() == true && spinCase == 1) {
      setWasHitLeft(false);
      stop();
    }

    if (spinCase == 0 && (getWasHitLeft() == false || isLeftLimit() != false)) {
      setTurretSpeedRaw(speed);

    } else if (getWasHitLeft() == true && spinCase == 0) {
      setWasHitRight(true);
      stop();
    }
  }

  public void stop() {
    turretSpinner.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run1
    // SmartDashboard.putBoolean("isLeftLimit", isLeftLimit());
    // SmartDashboard.putBoolean("isRightLimit", isRightLimit());

    // SmartDashboard.putBoolean("wasHitLeft", wasHitLeft);
    // SmartDashboard.putBoolean("wasHitRight", wasHitRight);
  }
}
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
  private CANDigitalInput limitLeft, limitRight;
  // private CANEncoder encoder;
  public boolean wasHitLeft;
  public boolean wasHitRight;

  public Turret() {
    turretSpinner = new CANSparkMax(Constants.TurretConstants.ID_TURRET, MotorType.kBrushless);
    // counter = new Counter(Constants.TurretConstants.DIO_TURRET);
    // leftLimit = new DigitalInput(Constants.TurretConstants.DIO_LEFT_LIMIT);
    // rightLimit = new DigitalInput(Constants.TurretConstants.DIO_RIGHT_LIMIT);
    
    // turretSpinner.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
    // LimitSwitchNormal.NormallyOpen, 0);
    // turretSpinner.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
    // LimitSwitchNormal.NormallyOpen, 1);
    limitLeft = turretSpinner.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    limitRight = turretSpinner.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

  }

  public void setTurretSpeed(double speed) {
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
    limitLeft.enableLimitSwitch(false);
    return limitLeft.get();
  }

  public boolean isRightLimit() {
    limitRight.enableLimitSwitch(false);
    return limitRight.get();
  }

  public void setSpeed(double speed) {
    if (!isLeftLimit() && !isRightLimit()) {
      setTurretSpeed(speed);
    } else {
      stop();
    }
  }

  public void set(double speed, int spinCase) {
    if(isLeftLimit() == false)
    {
      wasHitLeft = true;
    }
    if(isRightLimit() == false)
    {
      wasHitRight = true;
    }

    if(spinCase == 1 && (wasHitRight == false || isRightLimit() != true))
    {
      setTurretSpeed(speed);
    }
    else if(wasHitRight == true && spinCase == 1)
    {
      wasHitLeft = false;
      stop();
    }

    if(spinCase == 0 && (wasHitLeft == false || isLeftLimit() != true))
    {
      setTurretSpeed(-speed);

    }
    else if(wasHitLeft == true && spinCase == 0)
    {
      wasHitRight = false;
      stop();
    }
  }

  public void stop() {
    turretSpinner.set(0);
  }

  @Override
    public void periodic() {
      // This method will be called once per scheduler run1
      SmartDashboard.putBoolean("isLeftLimit", isLeftLimit());
      SmartDashboard.putBoolean("isRightLimit", isRightLimit());

      SmartDashboard.putBoolean("wasHitLeft", wasHitLeft);
      SmartDashboard.putBoolean("wasHitRight", wasHitRight);
    }
  }
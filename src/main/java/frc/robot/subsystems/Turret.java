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
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  public CANSparkMax turretSpinner;
  //private Counter counter;
  private DigitalInput leftLimit, rightLimit;
  //private CANEncoder encoder;

  public Turret() {
    turretSpinner = new CANSparkMax(Constants.TurretConstants.ID_TURRET, MotorType.kBrushless);
    //counter = new Counter(Constants.TurretConstants.DIO_TURRET);
    leftLimit = new DigitalInput(0);
    rightLimit = new DigitalInput(1);
    //turretSpinner.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    //turretSpinner.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 1);

  }
  public void setTurretSpeed(double speed) {
    turretSpinner.set(speed);
  }

  /*
  public void resetEncoders() {
    counter.reset();
    encoder.setPosition(0);
  }
  
  public double getEncoder() {
    return encoder.getPosition();
  }
  */
  public double getRawSpeed() {
    return turretSpinner.getEncoder().getVelocity();
  }

  public boolean isLeftLimit() {
    return leftLimit.get();
    //return turretSpinner.getSensorCollection().isRevLimitSwitchClosed();
  }

  public boolean isRightLimit() {
    return rightLimit.get();
    //return turretSpinner.getSensorCollection().isFwdLimitSwitchClosed();

  }
  public void setSpeed(double speed) {
    if (!isLeftLimit() && !isRightLimit()) {
      setTurretSpeed(speed);
    } else {
      stop();
    }
  }
  public void stop() {
    turretSpinner.set(0);
  }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  }
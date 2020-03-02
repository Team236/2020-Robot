/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {

  private CANSparkMax master, follower;

  private TalonSRX hood;

  private CANPIDController pidController;
  private CANEncoder encoder;

  private DigitalInput hoodLimit;
  private boolean isLimitThere;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {

    // master = new CANSparkMax(ID_MASTER, MotorType.kBrushless);
    master = new CANSparkMax(ID_MASTER, MotorType.kBrushless);
    follower = new CANSparkMax(ID_FOLLOWER, MotorType.kBrushless);

    hood = new TalonSRX(ID_HOOD);
    
    master.restoreFactoryDefaults();

    // Sets master inverted
    master.setInverted(false);

    // Sets follower, inverted from master
    follower.follow(master, true);

    pidController = master.getPIDController();
    encoder = master.getEncoder();

    try {
      hoodLimit = new DigitalInput(DIO_HOOD_LIMIT);
    } catch (Exception e) {
      isLimitThere = false;
    }
  }

  /**
   * 
   * @param speed desired speed in RPM
   */
  public void setSetPoint(double speed) {
    pidController.setReference(speed, ControlType.kVelocity);
  }

  public void setP(double kP) {
    pidController.setP(kP);

  }

  public void setI(double kI) {
    pidController.setI(kI);

  }

  public void setD(double kD) {
    pidController.setD(kD);

  }

  public void setFF(double kF) {
    pidController.setFF(kF);

  }

  public void setOutputRange() {
    pidController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public double getEncoder() {
    return encoder.getPosition();
  }

  /**
   * Gets velocity from Neo encoder
   * @return Shooter velocity in RPM
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Directly sets speed of shooter motor using percent output
   * @param speed desired speed -1.0 to 1.0
   */
  public void setSpeedRaw(double speed) {
    master.set(speed);
  }

  public void setHoodRaw(double speed)  {
    hood.set(ControlMode.PercentOutput, speed);
  }

  public void stopHood()  {
    hood.set(ControlMode.PercentOutput, 0);
  }

  public void stop() {
    setSpeedRaw(0);
  }

  public void updateConstants() {
    
  }

  public double getHoodEncoder() {
    return hood.getSelectedSensorPosition();
  }

  public void resetHoodEncoder() {
    hood.setSelectedSensorPosition(0);
  }

  public boolean getHoodLimit() {
    if (isLimitThere) {
      return hoodLimit.get();
    } else {
      return false;
    }
  }

  public void setHoodSpeed(double speed) {
    if (getHoodLimit() && speed < 0) {
      setHoodRaw(speed);
    } else {
      resetHoodEncoder();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

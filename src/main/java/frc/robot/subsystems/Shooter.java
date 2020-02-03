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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import  static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {

  private CANSparkMax master, follower;

  private CANPIDController pidController;
  private CANEncoder encoder;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {

  master = new CANSparkMax(ID_MASTER, MotorType.kBrushless);
  master.restoreFactoryDefaults();
  master.setInverted(true);
  pidController =master.getPIDController();
  encoder =master.getEncoder();
  }

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
  public double getVelocity() {
    return encoder.getVelocity();
  }
  public void setSpeedRaw(double speed) {
    master.set(speed);
    }
    public void stop() {
      setSpeedRaw(0);
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

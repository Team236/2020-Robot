/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.turn.TurnInterface;

import static frc.robot.Constants.DriveConstants.*;

public class Drive extends SubsystemBase implements TurnInterface {

  // TODO currently uses built-in encoders -- require testing (as good as
  // external??)

  private CANSparkMax leftFront, leftRear, rightFront, rightRear;
  private CANEncoder leftEncoder, rightEncoder;
  private AHRS navx;

  private CANPIDController leftPID, rightPID;

  /**
   * Creates a new Drive.
   */
  public Drive() {

    leftFront = new CANSparkMax(ID_LEFT_FRONT, MotorType.kBrushless);
    leftRear = new CANSparkMax(ID_LEFT_REAR, MotorType.kBrushless);
    rightFront = new CANSparkMax(ID_RIGHT_FRONT, MotorType.kBrushless);
    rightRear = new CANSparkMax(ID_RIGHT_REAR, MotorType.kBrushless);

    leftRear.follow(leftFront, false);
    rightRear.follow(rightFront, false);

    leftFront.setInverted(false);
    rightFront.setInverted(true);

    leftPID = leftFront.getPIDController();
    rightPID = rightFront.getPIDController();

    leftEncoder = leftFront.getEncoder();
    rightEncoder = rightFront.getEncoder();

  }

  /**
   * Sets left drive speed
   * 
   * @param speed
   */
  public void setLeftSpeed(double speed) {
    leftFront.set(speed);
  }

  /**
   * Sets right drive speed
   * 
   * @param speed
   */
  public void setRightSpeed(double speed) {
    rightFront.set(speed);
  }

  /**
   * Bumps speed to zero if between -0.1 to 0.1
   * 
   * @param speed
   */
  public void setLeftSpeedWithDeadzone(double speed) {
    double leftSpeed = speed;
    if (leftSpeed < LEFT_DEADZONE && leftSpeed > -LEFT_DEADZONE) {
      leftSpeed = 0;
    }

    setLeftSpeed(leftSpeed);
  }

  /**
   * Bumps speed to zero if between -0.1 to 0.1
   * 
   * @param speed
   */
  public void setRightSpeedWithDeadzone(double speed) {
    double rightSpeed = speed;
    if (rightSpeed < RIGHT_DEADZONE && rightSpeed > -RIGHT_DEADZONE) {
      rightSpeed = 0;
    }

    setRightSpeed(rightSpeed);
  }

  public double getGyroAngle() {
    return navx.getAngle();
    // return 0;
  }

  @Override
  public void resetAngle() {
    navx.reset();
  }

  @Override
  public double getRate() {
    return navx.getRate();
    // return 0;
  }

  /**
   * Sets both left/right drive speeds to zero
   */
  public void stop() {
    setLeftSpeed(0);
    setRightSpeed(0);
  }

  /**
   * Gets value of left Neo encoder
   * 
   * @return position of encoder
   */
  public double getLeftEncoder() {
    return leftEncoder.getPosition() * Constants.DriveConstants.REV_TO_IN_K;
  }

  /**
   * Gets value of right Neo encoder
   * 
   * @return position of encoder
   */
  public double getRightEncoder() {
    // rightFront.getEncoder().setPositionConversionFactor(-1);
    return rightEncoder.getPosition() * Constants.DriveConstants.REV_TO_IN_K;
  }

  /**
   * Resets Neo drive motor encoders
   */
  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public double getLeftSpeed() {
    return leftEncoder.getVelocity();
  }

  public double getRightSpeed() {
    return rightEncoder.getVelocity();
  }

  // PID
  @Override
  public double pidGet() {
    return getGyroAngle();
  }

  @Override
  public void pidSet(double speed) {
    setRightSpeed(speed);
    setLeftSpeed(-speed);
  }

  // SPARK MOTION CONTROL
  /**
   * Sets spark motion control kP
   * @param kP
   */
  public void setkP(double kP) {
    leftPID.setP(kP);
    rightPID.setP(kP);
  }

  public void setkI(double kI) {
    leftPID.setI(kI);
    rightPID.setI(kI);
  }

  public void setkD(double kD) {
    leftPID.setD(kD);
    rightPID.setD(kD);
  }

  public void setkF(double kF) {
    leftPID.setFF(kF);
    rightPID.setFF(kF);
  }

  /**
   * Sets min and max output for closed-loop control
   * 
   * @param minOutput
   * @param maxOutput
   */
  public void setOutputRange(double minOutput, double maxOutput) {
    leftPID.setOutputRange(minOutput, maxOutput);
    rightPID.setOutputRange(minOutput, maxOutput);
  }

  /**
   * Configures Spark Max to position-based closed-loop control mode and sets target distance
   * 
   * @param dist the target distance in inches
   */
  public void setSetPoint(double dist) {
    leftPID.setReference((dist * IN_TO_REV_K), ControlType.kPosition);
    rightPID.setReference((dist * IN_TO_REV_K), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left", getLeftEncoder());
    SmartDashboard.putNumber("Right", getRightEncoder());
  }
}

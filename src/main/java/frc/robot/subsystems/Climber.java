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
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
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
  private boolean isLimitUnplugged = false;

  private Relay lockRelay;

  private CANPIDController pidController;
  private CANEncoder encoder;

  /**
   * Creates a new Climber.
   */
  public Climber() {

    master = new CANSparkMax(ID_MASTER, MotorType.kBrushless);
    master.setInverted(true);

    lockRelay = new Relay(RELAY_PORT);

    bottomLimit = master.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    pidController = master.getPIDController();
    encoder = master.getEncoder();

    // Limit switch try catch
    try {
      newLimit = new DigitalInput(DIO_NEW_LIMIT);
    } catch (Exception e) {
      isLimitUnplugged = true;
    }

  }

  /**
   * Sets speed w/out consideration for limits
   * 
   * @param speed
   */
  private void setSpeedRaw(double speed) {
    master.set(speed);
  }
 /**
  * Reads limit as hooked to DIO
  * @return
  */
  public boolean isNewLimit() {
    if (isLimitUnplugged) {
      return false;
    } else {
      return !newLimit.get();
    }
  }

  /**
   * Sets pre-determined speed, takes into account limit
   */
  public void setSpeed(double speed) {
    // TODO add encoder upper limit
    // && getEncoderPosition() < ENC_LIMIT
    if (!isNewLimit() || speed > 0) {
      if (speed > 0 && getEncoderPosition() > ENC_LIMIT) {
        stop();
      } else {
        setSpeedRaw(speed);
      }
    } else {
      stop();
      resetEncoder();
    }
  }

  public void stop() {
    setSpeedRaw(0);
  }

  /**
   * old limit style, right now unused
   * @return
   */
  public boolean isBottomLimit() {
    bottomLimit.enableLimitSwitch(false);
    return bottomLimit.get();
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  // Unused pid methods
  public void setSetPoint(double position) {
    pidController.setReference(position, ControlType.kPosition);
  }

  public void setP(double kP) {
    pidController.setP(kP);
  }

  public void setI(double kI) {
    pidController.setP(kI);
  }

  public void setD(double kD) {
    pidController.setP(kD);
  }

  public void setFF(double kFF) {
    pidController.setFF(kFF);
  }

  public void setOutputRange(double min, double max) {
    pidController.setOutputRange(min, max);
  }

  /**
   * Powers relay: LL on, climb lock off
   */
  public void relayOn() {
    lockRelay.set(Value.kOn);
    lockRelay.set(Value.kForward);
  }

  /**
   * Cuts power to relay: LL off, climb lock engages
   */
  public void relayOff() {
    lockRelay.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("cl enc", getEncoderPosition());
  }
}

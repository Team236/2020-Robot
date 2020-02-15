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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {

  private TalonSRX master;
  private VictorSPX follower;
  private DigitalInput topLimit, bottomLimit;

  /**
   * Creates a new Climber.
   */
  public Climber() {

    master = new TalonSRX(ID_MASTER);
    follower = new VictorSPX(ID_FOLLOWER);

    follower.follow(master);

    // topLimit = new DigitalInput(DIO_TOP_LIMIT);
    // bottomLimit = new DigitalInput(DIO_BOT_LIMIT);

  }

  private void setSpeedRaw(double speed) {
    master.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    setSpeedRaw(0);
  }

  public boolean isTopLimit() {
    // return topLimit.get();
    return false;
  }

  public boolean isBottomLimit() {
    // return bottomLimit.get();
    return false;
  }

  public double getEncoder() {
    return master.getSelectedSensorPosition();
  }

  public void resetEncoder() {
    master.setSelectedSensorPosition(0);
  }

  /**
   * Sets pre-determined speed, takes into account limits
   */
  public void setSpeed(double speed) {
    if (isTopLimit() || isBottomLimit()) {
      stop();
    } else {
      setSpeedRaw(speed);
    }
  }

  /**
   * Sets to specified speed unless past encoder limit
   */
  public void setSpeedEnc() {
    if (getEncoder() < ENC_LIMIT) {
      setSpeed(SPEED);
    } else {
      setSpeed(0.2);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

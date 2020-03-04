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

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {

  private VictorSPX intakeMotor;
  private TalonSRX raiseLowerMotor;
  private DigitalInput upperLimit, lowerLimit;

  private Counter ballCounter;
  private boolean isCounterUnplugged = false;
  private boolean limitsUnplugged = false;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    intakeMotor = new VictorSPX(ID_MOTOR);
    raiseLowerMotor = new TalonSRX(ID_POSITION_MOTOR);
    raiseLowerMotor.setInverted(false);

    // Limit switches
    try {
      upperLimit = new DigitalInput(DIO_UPPER_LIMIT);
      lowerLimit = new DigitalInput(DIO_LOWER_LIMIT);

    } catch (Exception e) {
      limitsUnplugged = true;
    }

    // Ball counter
    try {
      this.ballCounter = new Counter();
      this.ballCounter.setUpSource(DIO_INTAKE_COUNTER);
      this.ballCounter.setDownSource(Constants.ShooterConstants.DIO_SHOOT_COUNTER);
    } catch (Exception e) {
      isCounterUnplugged = true;
    }

  }

  public void setSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, -speed);
  }

  public void stop() {
    setSpeed(0);
  }

  /**
   * Reads ball counter (intake & shooter counters)
   * 
   * @return number of balls in robot
   */
  public int getBallCount() {
    if (isCounterUnplugged) {
      return 0;
    } else {
      return ballCounter.get();
    }
  }

  /**
   * Resets intake counter to 0
   */
  public void resetCounter() {
    ballCounter.reset();
  }

  public boolean getUpperLimit() {
    if (limitsUnplugged) {
      return false;
    } else {
      return upperLimit.get();
    }

  }

  public boolean getLowerLimit() {
    if (limitsUnplugged) {
      return false;
    } else {
      return lowerLimit.get();
    }
  }

  /**
   * Sets speed of motor that positions intake up/down
   * 
   * @param speed
   */
  public void setPositionSpeed(double speed) {
    if (speed > 0 && getUpperLimit()) {
      stopPositionMotor();
    } else if (speed < 0 && getLowerLimit()) {
      stopPositionMotor();
    } else {
      raiseLowerMotor.set(ControlMode.PercentOutput, speed);
    }

  }

  public void raise() {

  }

  public void lower() {

  }

  public void stopPositionMotor() {
    setPositionSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("intake upper", getUpperLimit());
  }
}

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {

  private VictorSPX intakeMotor;
  private TalonSRX raiseLowerMotor;

  private Counter ballCounter;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    intakeMotor = new VictorSPX(ID_MOTOR);
    raiseLowerMotor = new TalonSRX(ID_POSITION_MOTOR);

    this.ballCounter = new Counter();
    this.ballCounter.setUpSource(DIO_INTAKE_COUNTER);
    this.ballCounter.setDownSource(Constants.ShooterConstants.DIO_SHOOT_COUNTER);

  }

  public void setSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
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
    return ballCounter.get();
  }

  /**
   * Resets intake counter to 0
   */
  public void resetCounter() {
    ballCounter.reset();
  }

  /**
   * Sets speed of motor that positions intake up/down
   * 
   * @param speed
   */
  public void setPositionSpeed(double speed) {
    raiseLowerMotor.set(ControlMode.PercentOutput, speed);
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
  }
}

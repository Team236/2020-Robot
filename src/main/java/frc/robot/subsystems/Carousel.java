/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CarouselConstants.*;

public class Carousel extends SubsystemBase {

  private VictorSPX carouselMotor;
  private VictorSPX rollerMotor;

  private Servo toShootServo;
  private Servo toShootServo2;

  /**
   * Creates a new Carousel.
   */
  public Carousel() {
    carouselMotor = new VictorSPX(ID_MOTOR);
    rollerMotor = new VictorSPX(ID_ROLLER_MOTOR);

    toShootServo = new Servo(PWM_TO_SHOOT_SERVO);
    toShootServo2 = new Servo(PWM_TO_SHOOT_SERVO_2);
  }

  /**
   * Sets speed of carousel motor
   * @param speed
   */
  public void setSpeed(double speed) {
    carouselMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    setSpeed(0);
  }

  /**
   * Spins rollers that push ball to shooter
   * @param speed
   */
  public void setRollerSpeed(double speed) {
    rollerMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopRoller() {
    setRollerSpeed(0);
  }

  /**
   * Sets both servos that push ball to shooter (green wheel)
   * @param position desired position of servo
   */
  public void setToShootServos(double position) {
    toShootServo.setPosition(position);
    toShootServo2.setPosition(position);
  }

  /**
   * Extends green wheel to bump ball to rollers
   */
  public void extendBumpWheel() {
    setToShootServos(EXTEND_POS);
  }

  /**
   * Retracts green wheel
   */
  public void retractBumpWheel() {
    setToShootServos(RETRACT_POS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

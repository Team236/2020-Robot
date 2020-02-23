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

  private Servo toShootServo;

  /**
   * Creates a new Carousel.
   */
  public Carousel() {
    carouselMotor = new VictorSPX(ID_MOTOR);
    toShootServo = new Servo(PWM_TO_SHOOT_SERVO);
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
   * Sets servo that pushes ball to shooter (from carousel)
   * @param position desired position of servo
   */
  public void setToShootServo(double position) {
    toShootServo.setPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

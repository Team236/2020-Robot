/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CarouselConstants.*;

public class Carousel extends SubsystemBase {

  private TalonSRX carouselMotor;

  /**
   * Creates a new Carousel.
   */
  public Carousel() {
    carouselMotor = new TalonSRX(ID_MOTOR);
  }

  public void setSpeed(double speed) {
    carouselMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    setSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

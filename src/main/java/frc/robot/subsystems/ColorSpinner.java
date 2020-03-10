/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ColorSpinnerConstants.*;

public class ColorSpinner extends SubsystemBase {

  // Vex servos- about 76 rpm

  private CANSparkMax spinnerMotor;
  private Servo servo;
  private Servo servo2;

  private ColorSensorV3 colorSensor;
  private ColorMatch colorMatcher;
  private Color kBlueTarget, kGreenTarget, kRedTarget, kYellowTarget;
  private String colorString;

  private String gameData;
  private char fmsColor;

  /**
   * Creates a new ColorSpinner.
   */
  public ColorSpinner() {
    spinnerMotor = new CANSparkMax(ID_MOTOR, MotorType.kBrushless);
    servo = new Servo(PWM_SERVO);
    servo2 = new Servo(PWM_SERVO_2);

    servo.setBounds(2.1, 2.1, 1.5, .95, .95);
    servo2.setBounds(2.1, 2.1, 1.5, .95, .95);

    I2C.Port i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);
    colorMatcher = new ColorMatch();

    kBlueTarget = BLUE;
    kGreenTarget = GREEN;
    kRedTarget = RED;
    kYellowTarget = YELLOW;

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
    colorMatcher.addColorMatch(kGreenTarget);

  }

  /**
   * Sets speed to turn mechanism's wheel
   * 
   * @param speed
   */
  public void setSpeed(double speed) {
    spinnerMotor.set(speed);
  }

  /**
   * Stops mechanism wheel
   */
  public void stop() {
    setSpeed(0);;
  }

  /**
   * Determines color currently in view
   * 
   * @return The current color
   */
  public String getCurrentColor() {

    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Confidence Color Match", match.confidence);
    SmartDashboard.putNumber("Red", colorSensor.getRed());
    SmartDashboard.putNumber("Green", colorSensor.getGreen());
    SmartDashboard.putNumber("Blue", colorSensor.getBlue());

    return colorString;
  }

  /**
   * Determines whether current color and previous color are the same
   * 
   * @param prevColor
   * @param nextColor
   * @return True if color has changed
   */
  public boolean isColorChange(String prevColor, String nextColor) {
    return !prevColor.equals(nextColor);
  }

  /**
   * Retrieves desired color from FMS
   * 
   * @return Color requested by FMS
   */
  public String getDesiredColor() {

    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {

      fmsColor = gameData.charAt(0);

      if (fmsColor == 'B') {
        return "Red";
      }
      if (fmsColor == 'G') {
        return "Yellow";
      }
      if (fmsColor == 'R') {
        return "Blue";
      }
      if (fmsColor == 'Y') {
        return "Green";
      } else {
        return "";
      }
    } else {
      return "";
    }
  }

  /**
   * Compares current color and desired color
   * 
   * @param currentColor
   * @param desiredColor
   * @return True when current color and desired color are equal
   */
  public boolean atDesiredColor(String currentColor, String desiredColor) {
    return currentColor.equals(desiredColor);
  }

  /**
   * Gets proximity value: larger when closer
   * 
   * @return Proximity measurement value, ranging from 0 to 2047
   */
  public int getColorProimity() {
    return colorSensor.getProximity();
  }

  /**
   * Gets servo position
   * 
   * @return position between 0.0 and 1.0
   */
  public double getServoPos() {
    return servo.get();
  }

  /**
   * Extends both color spinner servos (folds down mechanism)
   */
  public void extend() {
    servo.set(EXTEND_VALUE);
    servo2.set(EXTEND_VALUE);
  }

  /**
   * Retracts both color spinner servos (pops up mechanism)
   */
  public void retract() {
    servo.set(RETRACT_VALUE);
    servo2.set(RETRACT_VALUE);
  }

  public void stopServo() {
    servo.set(0.5);
    servo2.set(0.5);
  }

  /**
   * 
   * @return true if servo is extended past EXTENDED_VALUE constant
   */
  public boolean isExtended() {

    if (getServoPos() >= (EXTEND_VALUE)) {
      return true;
    } else {
      return false;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("cs servo", getServoPos());
    // SmartDashboard.putNumber("bounds", servo.getRawBounds());
    // System.out.println(servo.getRawBounds());
  }
}

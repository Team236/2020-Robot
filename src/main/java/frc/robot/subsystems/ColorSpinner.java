/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ColorSpinner extends SubsystemBase {

  public VictorSPX spinnerMotor;

  public ColorSensorV3 colorSensor;
  ColorMatch colorMatcher;
  Color kBlueTarget, kGreenTarget, kRedTarget, kYellowTarget;
  public String colorString;

  String gameData;
  char fmsColor;
  /**
   * Creates a new ColorSpinner.
   */
  public ColorSpinner() {
    spinnerMotor = new VictorSPX(Constants.ColorSpinnerConstants.MOTOR_ID);

    I2C.Port i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);
    colorMatcher = new ColorMatch();

    kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
    colorMatcher.addColorMatch(kGreenTarget);

  }

  public void startMotor(double speed) {
    spinnerMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopMotor() {
    spinnerMotor.set(ControlMode.PercentOutput, 0);
  }

  public String getColor() {

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

  public boolean isColorChange(String prevColor, String nextColor) {
    return !prevColor.equals(nextColor);
  }

  public String desiredColor() {

    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0) {

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
      }
      else {
        return "";
      }
    }
    else {
      return "";
    }
  }

  public boolean atDesiredColor(String currentColor, String desiredColor) {
    return currentColor.equals(desiredColor);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

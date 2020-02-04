/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ColorSpinner;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSpinner;

public class ColorSpinnerPosition extends CommandBase {
  
  private String currentColor;
  private String desiredColor;

  private ColorSpinner colorSpinnerPosition;
  /**
   * Creates a new ColorSpinnerPosition.
   */
  public ColorSpinnerPosition(ColorSpinner _colorSpinner) {
    // Use addRequirements() here to declare subsystem dependencies.
    colorSpinnerPosition = _colorSpinner;
    addRequirements(colorSpinnerPosition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentColor = colorSpinnerPosition.getColor();
    desiredColor = colorSpinnerPosition.desiredColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    colorSpinnerPosition.startMotor(Constants.ColorSpinnerConstants.SPEED);
    currentColor = colorSpinnerPosition.getColor();
    SmartDashboard.putString("desired color", desiredColor);
    SmartDashboard.putString("current color", currentColor);
    SmartDashboard.putNumber("Color Match", colorSpinnerPosition.colorSensor.getProximity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorSpinnerPosition.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentColor == desiredColor;
  }
}

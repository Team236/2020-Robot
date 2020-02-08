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

  private ColorSpinner colorSpinner;

  private String currentColor;
  private String desiredColor;

  /**
   * Creates a new ColorSpinnerPosition.
   */
  public ColorSpinnerPosition(ColorSpinner colorSpinner) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.colorSpinner = colorSpinner;
    addRequirements(this.colorSpinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentColor = colorSpinner.getCurrentColor();
    desiredColor = colorSpinner.getDesiredColor();

    SmartDashboard.putString("desired color", desiredColor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    colorSpinner.startMotor(Constants.ColorSpinnerConstants.SPEED);
    currentColor = colorSpinner.getCurrentColor();

    SmartDashboard.putString("current color", currentColor);
    // SmartDashboard.putNumber("proximity", colorSpinner.getColorProimity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorSpinner.stopMotor();
  }

  // Returns true when current color and desired color are the same
  @Override
  public boolean isFinished() {
    return colorSpinner.atDesiredColor(currentColor, desiredColor);
  }
}

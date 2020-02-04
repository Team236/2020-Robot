/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ColorSpinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSpinner;

public class ColorSpinnerRotation extends CommandBase {

  private ColorSpinner colorSpinnerRotatation;

  private String prevColor;
  private String nextColor;
  private int i;
  /**
   * Creates a new ColorSpinnerRotation.
   */
  public ColorSpinnerRotation(ColorSpinner colorSpinner) {
    // Use addRequirements() here to declare subsystem dependencies.
    colorSpinnerRotatation = colorSpinner;
    addRequirements(colorSpinnerRotatation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
    prevColor = colorSpinnerRotatation.getColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    colorSpinnerRotatation.startMotor(Constants.ColorSpinnerConstants.SPEED);
    nextColor = colorSpinnerRotatation.getColor();
 
    System.out.println("i before if " + i);
    System.out.println(colorSpinnerRotatation.getColor());
    if (colorSpinnerRotatation.isColorChange(prevColor, nextColor)) {
     i++;
     prevColor = colorSpinnerRotatation.getColor();     
    }
    System.out.println("i after if " + i);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorSpinnerRotatation.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return i > Constants.ColorSpinnerConstants.COLOR_COUNT;
  }
}

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

public class ColorSpinnerRotation extends CommandBase {

  private ColorSpinner colorSpinner;

  private String prevColor;
  private String nextColor;
  private int i;
  /**
   * Creates a new ColorSpinnerRotation.
   */
  public ColorSpinnerRotation(ColorSpinner colorSpinner) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.colorSpinner = colorSpinner;
    addRequirements(this.colorSpinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
    prevColor = colorSpinner.getColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    colorSpinner.startMotor(Constants.ColorSpinnerConstants.SPEED);
    nextColor = colorSpinner.getColor();
 
    System.out.println("i before if " + i);
    System.out.println(colorSpinner.getColor());
    if (colorSpinner.isColorChange(prevColor, nextColor)) {
     i++;
     prevColor = colorSpinner.getColor();     
    }
    System.out.println("i after if " + i);

    SmartDashboard.putNumber("color count", i);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorSpinner.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return i > Constants.ColorSpinnerConstants.COLOR_COUNT;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ColorSpinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSpinner;

public class ColorSpinnerRetract extends CommandBase {

  private ColorSpinner colorSpinner;

  /**
   * Retracts Color Spinner using linear actuator
   */
  public ColorSpinnerRetract(ColorSpinner colorSpinner) {
    this.colorSpinner = colorSpinner;
    addRequirements(this.colorSpinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorSpinner.retract();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

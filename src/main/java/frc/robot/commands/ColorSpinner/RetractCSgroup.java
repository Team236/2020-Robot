/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ColorSpinner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSpinner;
import lib.commands.Wait;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RetractCSgroup extends ParallelDeadlineGroup {
  /**
   * cs down.
   */
  public RetractCSgroup(ColorSpinner colorSpinner) {
    // Add your commands in the super() call.  Add the deadline first.
    super(
      new Wait(Constants.ColorSpinnerConstants.RETRACT_TIME),
      new ColorSpinnerRetract(colorSpinner)
    );
  }
}

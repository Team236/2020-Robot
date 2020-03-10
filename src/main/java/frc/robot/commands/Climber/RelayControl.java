/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RelayControl extends CommandBase {
  private Climber climber;
  private boolean isDisengage;
  /**
   * Creates a new RelayControl.
   */
  public RelayControl(Climber climber, boolean isDisengage) {
    this.climber = climber;
    this.isDisengage = isDisengage; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (isDisengage) {
      climber.relayOff();
    } else {
      climber.relayOn();
    }

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

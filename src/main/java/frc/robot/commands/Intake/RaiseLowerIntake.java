/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RaiseLowerIntake extends CommandBase {

  private Intake intake;
  private boolean isRaise;

  /**
   * @param direction set to True to raise
   */
  public RaiseLowerIntake(Intake intake, boolean direction) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
    this.isRaise = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isRaise) {
      intake.setPositionSpeed(Constants.IntakeConstants.RAISE_SPEED);
    } else {
      intake.setPositionSpeed(Constants.IntakeConstants.LOWER_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopPositionMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO run on whenHeld and have it finish when limit is hit or current spikes
    if (isRaise && intake.getUpperLimit()) {
        return true;
    } else if (!isRaise && intake.getLowerLimit()) {
      return true;
    } else {
      return false;
    }
  }
}

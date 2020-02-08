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

public class SetIntakeSpeed extends CommandBase {

  private Intake intake;
  private double speed;

  /**
   * Sets intake speed
   */
  public SetIntakeSpeed(Intake intake, double speed) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);

    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If considering count, only sets speed when count < max
    if (Constants.IntakeConstants.CONSIDER_COUNT) {
      if (intake.getBallCount() < Constants.IntakeConstants.MAX_COUNT) {
        intake.setSpeed(speed);
      } else {
        intake.stop();
      }
    } else {
      intake.setSpeed(speed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

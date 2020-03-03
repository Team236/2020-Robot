/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class SetClimbSpeed extends CommandBase {

  private Climber climber;
  private boolean isFwd;

  /**
   * Creates a new SetClimbSpeed.
   */
  public SetClimbSpeed(Climber climber, boolean isFwd) {
    this.climber = climber;
    this.isFwd = isFwd;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isFwd) {
      climber.setSpeed(Constants.ClimberConstants.SPEED);
      // climber.setSpeedEnc();
    } else {
      climber.setSpeed(-Constants.ClimberConstants.SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
    climber.relayOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}

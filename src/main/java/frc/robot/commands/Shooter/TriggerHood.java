/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.ShooterConstants.*;

public class TriggerHood extends CommandBase {
  private Shooter hood;
  private int spinCase;
  private int trip;
  /**
   * Creates a new TriggerHood.
   */
  public TriggerHood(Shooter _hood, int _case) {
    this.hood = _hood;
    this.spinCase = _case;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(spinCase == 0) {
      hood.setHoodRaw(HOOD_SPEED);

    }
    else if(spinCase == 1)  {
      hood.setHoodRaw(-HOOD_SPEED);
    }
/*
    if(hood.getHoodLimit() == 1 && trip != 1)  {
      hood.stopHood();
      trip = 1;
    }
*/
    //hood.resetHoodEncoder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

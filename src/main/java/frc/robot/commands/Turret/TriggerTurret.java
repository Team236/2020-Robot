 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Turret;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;
import static frc.robot.Constants.TurretConstants.*;


public class TriggerTurret extends CommandBase {
  private Turret turret;
  private int spinCase;
  private boolean wasHitRight, wasHitLeft;
  
  /**
   * Creates a new ReplaceMeCommand.
   */
  public TriggerTurret(Turret _turret, int _case) {
    this.turret = _turret;
    this.spinCase = _case;
    
    // addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wasHitLeft = false;
    wasHitRight = false;
/*
    turret.setWasHitLeft(false);
    turret.setWasHitRight(false);
    */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    if (turret.isLeftLimit() == false) {
      wasHitLeft = true;
    }
    if (turret.isRightLimit() == false) {
      wasHitRight = true;
    }
    
    if (spinCase == 0 && (wasHitLeft == false || turret.isLeftLimit() != false)) {
      turret.setTurretSpeedRaw(Constants.TurretConstants.TURRET_SPEED);
    } else if (wasHitLeft == true && spinCase == 0) {
      wasHitRight = false;
      turret.stop();
    }
    
    if (spinCase == 1 && (wasHitRight == false || turret.isRightLimit() != false)) {
      turret.setTurretSpeedRaw(-Constants.TurretConstants.TURRET_SPEED);
    } else if (wasHitRight == true && spinCase == 1) {
      wasHitLeft = false;
      turret.stop();
    }*/
    
    SmartDashboard.putBoolean("isLeftLimit", turret.isLeftLimit());
    SmartDashboard.putBoolean("isRightLimit", turret.isRightLimit());

    SmartDashboard.putBoolean("wasHitLeft", wasHitLeft);
    SmartDashboard.putBoolean("wasHitRight", wasHitRight);
    //SmartDashboard.putBoolean("smack", smack);

    //SmartDashboard.putNumber("Encoder Position", turret.getEncoder());

    turret.set(TURRET_SPEED, spinCase);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;
import static frc.robot.Constants.TurretConstants.*;

import lib.limelightLib.ControlMode.LedMode;

public class ClimberVision extends CommandBase {
  private Limelight lime;
  private Turret turret;
  private Shooter hood;

  /**
   * Creates a new ClimberVision.
   */
  public ClimberVision(Limelight limeSub, Turret turretSub, Shooter shooterSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lime = limeSub;
    this.turret = turretSub;
    this.hood = shooterSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lime.setLEDMode(LedMode.kforceOff);
    lime.setPipeline(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.set(TURRET_SPEED, 0);
    // if(hood.getHoodLimit() == false)  {
      hood.setHoodSpeed(.8);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
    hood.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.Shooter;;

public class SparkShoot2 extends CommandBase {

  private Shooter shooterSub;
  private double speed;
  /**
   * Creates a new ShooterSparkControl.
   */
  public SparkShoot2(Shooter shooterSub, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSub = shooterSub;
    addRequirements(this.shooterSub);
    
    this.speed = speed;
  }

  // public ShooterSparkControl(NeoShooter shooterSub) {
    // this.shooterSub = shooterSub;
    // addRequirements(this.shooterSub);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSub.resetEncoder();

    shooterSub.setP(kP);
    shooterSub.setI(kI);
    shooterSub.setD(kD);
    shooterSub.setFF(kFF);
    shooterSub.setOutputRange();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSub.setSetPoint(speed);

    // System.out.println("shooter spark pid execute");

    SmartDashboard.putNumber("shooter setpoint", speed);
    SmartDashboard.putNumber("actual speed", shooterSub.getVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

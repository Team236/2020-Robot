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

public class ShooterSparkControl extends CommandBase {

  private Shooter shooter;
  private double speed;

  // comment these three lines after tuning PID
  private double s_ff, s_pp, s_ii, s_dd;
  private double s_kF, s_kP, s_kI, s_kD;
  private double s_lastf, s_lastp, s_lasti, s_lastd;
  /**
   * Creates a new ShooterSparkControl.
   */
  public ShooterSparkControl(Shooter shooter, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(this.shooter);

    this.speed = speed;

    // comment these five lines after tuning PID
    s_kF = 0;  s_kP = 0; s_kI = 0; s_kD = 0;
    SmartDashboard.putNumber("Shooter Feed Fwd", s_lastf);
    SmartDashboard.putNumber("Shooter k_P", s_lastp);
    SmartDashboard.putNumber("Shooter k_I", s_lasti);
    SmartDashboard.putNumber("Shooter k_I", s_lastd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.resetEncoder();   
    
    shooter.setOutputRange();
    
    // comment these four rows when PID tuning done
    shooter.setP(s_kP);
    shooter.setI(s_kI);
    shooter.setD(s_kD);
    shooter.setFF(s_kF);

    // uncomment below when PID tuning is done
 // shooterSub.setP(Constants.ShooterConstants.kP);
  // shooterSub.setI(Constants.ShooterConstants.kI);
  // shooterSub.setD(Constants.ShooterConstants.kD);
  //shooterSub.setFF(Constants.ShooterConstants.kFF); 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //comment these four lines when PID tuning done
    s_ff = SmartDashboard.getNumber("Shooter Feed Fwd", 0);
    s_pp = SmartDashboard.getNumber("Shooter k_P", 0);
    s_ii = SmartDashboard.getNumber("Shooter k_I", 0);
    s_dd = SmartDashboard.getNumber("Shooter k_D", 0);
    //comment these eight lines when PID tuning done
    if ((s_ff != s_kF)) {shooter.setFF(s_ff); s_kF = s_ff;}
    SmartDashboard.putNumber("set s_ff", s_ff);
    if ((s_pp != s_kP)) {shooter.setP(s_pp); s_kP = s_pp;}
    SmartDashboard.putNumber("set s_pp", s_pp);
    if ((s_ii != s_kI)) {shooter.setI(s_ii); s_kI = s_ii;}
    SmartDashboard.putNumber("set s_ii", s_ii);
    if ((s_dd != s_kD)) {shooter.setP(s_dd); s_kD = s_dd;}
    SmartDashboard.putNumber("set s_dd", s_dd);

    shooter.setSetPoint(speed);
    System.out.println("shooter spark pid execute");

    SmartDashboard.putNumber("shooter setpoint", speed);
    SmartDashboard.putNumber("actual speed", shooter.getVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();

    //comment these four lines when PID tuning done
    s_lastf = SmartDashboard.getNumber("Shooter Feed Fwd", 0);
    s_lastp = SmartDashboard.getNumber("Shooter k_P", 0);
    s_lasti = SmartDashboard.getNumber("Shooter k_I", 0);
    s_lastd = SmartDashboard.getNumber("Shooter k_D", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

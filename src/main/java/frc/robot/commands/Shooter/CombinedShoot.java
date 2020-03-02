/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;
import frc.robot.commands.Shooter.LimeSequentialShooter;
import frc.robot.commands.Turret.LimeLightTurret;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CombinedShoot extends ParallelCommandGroup {
  /**
   * Creates a new CombinedShoot.
   */
  
  public CombinedShoot(Shooter shooter, Limelight lime, Turret turret, double kP1, double kI1, double kD1, double kP2, double kI2, double kD2) {
    //PID 1: Hood
    //PID 2: Turret

    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(new LimeSequentialShooter(shooter, lime, kP1, kI1, kD1), new LimeLightTurret(lime, turret, kP2, kI2, kD2));
  }
}

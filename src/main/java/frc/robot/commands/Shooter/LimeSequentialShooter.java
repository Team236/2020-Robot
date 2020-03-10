/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.Shooter.LimeLightVerticalZero;
import frc.robot.commands.Shooter.LimeLightHoodOffset;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LimeSequentialShooter extends SequentialCommandGroup {
  /**
   * Creates a new LimeSequentialShooter.
   */
  public LimeSequentialShooter(Shooter shooter, Limelight lime, double kP1, double kI1, double kD1) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new LimeLightVerticalZero(lime, shooter, kP1, kI1, kD1), new LimeLightHoodOffset(lime, shooter, kP1, kI1, kD1));
  }
}

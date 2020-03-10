/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto.Routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Auto.SparkMotnControl;
import frc.robot.commands.Carousel.CarouselToShoot;
import frc.robot.commands.Intake.RaiseLowerIntake;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.commands.Shooter.ShooterSparkControl;
import frc.robot.commands.Turret.LimeLightTurret;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import lib.commands.TimeoutCommand;
import lib.commands.Wait;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootInt3Shoot extends SequentialCommandGroup {
  /**
   * Creates a new ShootInt3Shoot.
   */
  public ShootInt3Shoot(Drive drive, Shooter shooter, Carousel carousel, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    // TODO intake on way back
    super(parallel(sequence(new Wait(Constants.ShooterConstants.CAR_DELAY), new CarouselToShoot(carousel)),
     new ShooterSparkControl(shooter, Constants.ShooterConstants.HIGH_SPEED),
     new RaiseLowerIntake(intake, false)).withTimeout(3.5), 
     parallel(new SparkMotnControl(drive, 48, 3), new SetIntakeSpeed(intake)).withTimeout(2.8),
     parallel(new SparkMotnControl(drive, -48, 3), new SetIntakeSpeed(intake)).withTimeout(3),
     parallel(sequence(new Wait(Constants.ShooterConstants.CAR_DELAY), new CarouselToShoot(carousel)), 
     new ShooterSparkControl(shooter, Constants.ShooterConstants.HIGH_SPEED).withTimeout(3.5)));
  }
}

// carousel/feed, shoot, & lower intake
// drive to trench and intake
// drive back to init line
// carousel/feed, shoot

// parallel(new SparkMotnControl(drive, 48, 3), new SetIntakeSpeed(intake)).withTimeout(2.8)
// new SparkMotnControl(drive, -48, 3).withTimeout(2.8)
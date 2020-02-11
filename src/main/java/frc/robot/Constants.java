/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class ControllerConstants {
        public static final int USB_LEFT_STICK = 0;
        public static final int USB_RIGHT_STICK = 1;
        public static final int USB_CONTROLLER = 2;
    }

    public static class DriveConstants {
        public static final int ID_LEFT_FRONT = 7;
        public static final int ID_LEFT_REAR = 8;
        public static final int ID_RIGHT_FRONT = 4;
        public static final int ID_RIGHT_REAR = 6;

        public static final double LEFT_DEADZONE = .1;
        public static final double RIGHT_DEADZONE = .1;

        public static final boolean IS_DEADZONE = true;
    }

    public static class IntakeConstants {
        public static final int ID_MOTOR = 24;
        public static final int ID_POSITION_MOTOR = 99;
        public static final int DIO_INTAKE_SENSOR = 0;

        public static final double LIME_KP = .005;
        public static final double LIME_KI = .0;
        public static final double LIME_KD = .005;
        public static final double LIME_SPEED = 0.2;
        
        public static final double SPEED = .75;
        
        public static final boolean CONSIDER_COUNT = false;
        public static final int MAX_COUNT = 6;

        public static final double I_ACTIVE_ZONE = 3.00;
    }

    public static class ShooterConstants {
        public static final int ID_MASTER = 5;
        public static final int ID_FOLLOWER = 9;
        public static final int ID_HOOD = 0;

        public static final int DIO_SHOOT_COUNTER = 1;

        public static final double SPEED_RPM = 500;
        public static final double HOOD_SPEED = .10;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.0002;

        public static final double MIN_OUTPUT = 0.0;
        public static final double MAX_OUTPUT = 6000.0;
        public static final double interval = 0.0;

    }

    public static class ColorSpinnerConstants {
        public static final int ID_MOTOR = 14;
        public static final int PWM_SERVO = 0;

        public static final double SPEED = 0.05;

        public static final int COLOR_COUNT = 32;

        public static final double EXTEND_VALUE = 0.8;
        public static final double RETRACT_VALUE = 0.2;
    }

    public static class CarouselConstants {
        public static final int ID_MOTOR = 25;

        public static final double SPEED = .5;
    }

    public static class TurretConstants {
        public static final int ID_TURRET = 8;
        public static final int DIO_TURRET = 0;
        public static final double TURRET_kP = 0.03;
        public static final double TURRET_kI = 0.0045;
        public static final double TURRET_kD = 0.03;
        public static final double TURRET_SPEED = 0.2;
        public static final double I_ACTIVE_ZONE = 5.00;
    }

    public static class ClimberConstants {
        public static final int ID_MASTER = 0;
        public static final int ID_FOLLOWER = 0;

        public static final int DIO_TOP_LIMIT = 11;
        public static final int DIO_BOT_LIMIT = 11;

        public static final double SPEED = 1.0;
    }
}

package frc.robot.constants;

public class Constants {
    public static double DEADZONE_VALUE = 0.08;

    enum CurrentRobot {
        ROBOT, SIM
    }

    public static final CurrentRobot currentRobot = CurrentRobot.ROBOT;
}

package frc.robot.subsystems.example;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
    // put hardware here (TalonFX, CANSparkMax, etc.)
    // also put some variables used such as a state, goal, etc.

    private static ExampleSubsystem instance;

    private ExampleSubsystem() {
        // define hardware (do not make the subsystem constructor have any arguments
        // just use IDs defined in the constants file
        // also define the variables
    }

    public static ExampleSubsystem getInstance() {
        if (instance == null) {
            instance = new ExampleSubsystem();
        }
        return instance;
    }
}

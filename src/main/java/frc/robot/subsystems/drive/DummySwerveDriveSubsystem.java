package frc.robot.subsystems.drive;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DummySwerveDriveSubsystem implements ISwerveDriveSubsystem {
    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d();
    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {

    }

    @Override
    public Pose2d getPose() {
        return null;
    }

    @Override
    public Command pathfindCommand(Pose2d targetPose) {
        return null;
    }

    @Override
    public Command driveFieldCentricCommand() {
        return null;
    }

    @Override
    public Command followChoreoPath(ChoreoTrajectory trajectory, boolean resetPosition) {
        return null;
    }

    @Override
    public Command followChoreoPath(String pathName, boolean resetPosition) {
        return null;
    }
}

package frc.robot.subsystems.drive;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Controls;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.util.CommandsUtil;
import frc.robot.util.DriverStationUtil;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import static frc.robot.subsystems.drive.constants.DriveConstants.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem, so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements ISwerveDriveSubsystem, Sendable {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.FieldCentricFacingAngle SOTFRequest = new SwerveRequest.FieldCentricFacingAngle();

    private final Field2d field = new Field2d();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        seedFieldRelative(new Pose2d());
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SOTFRequest.HeadingController = new PhoenixPIDController(8, 0, 0);
        SOTFRequest.HeadingController.setTolerance(Rotation2d.fromDegrees(7.5).getRadians());
    }

    private SwerveRequest fieldCentricRequestSupplier() {
        double forwardsSpeed = Controls.DriverControls.SwerveForwardAxis.getAsDouble();
        double sidewaysSpeed = Controls.DriverControls.SwerveStrafeAxis.getAsDouble();
        double rotationSpeed = Controls.DriverControls.SwerveRotationAxis.getAsDouble();
        return fieldCentricRequest
                .withVelocityX(forwardsSpeed)
                .withVelocityY(sidewaysSpeed)
                .withRotationalRate(rotationSpeed);
    }

    public Command pathfindCommand(Pose2d targetPose) {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        PathConstraints constraints = new PathConstraints(
                6, 3,
                2 * Math.PI, 2 * Math.PI
        );
        return AutoBuilder.pathfindToPoseFlipped(
                targetPose,
                constraints
        );
    }

    public Command driveFieldCentricCommand() {
        return applyRequest(this::fieldCentricRequestSupplier);
    }

    private Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        field.setRobotPose(getPose());
        PoseEstimate pose = validatePoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight"));
        if (pose != null) {
            addVisionMeasurement(pose.pose, Timer.getFPGATimestamp());
        }
        SmartDashboard.putData("Drive/Field", field);
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.020, RobotController.getBatteryVoltage());
        field.setRobotPose(getPose());
        SmartDashboard.putData("Drive/Field", field);
    }

    public void reset() {
        m_pigeon2.reset();
    }

    private PoseEstimate validatePoseEstimate(PoseEstimate poseEstimate) {
        if (poseEstimate == null) return null;
        Pose2d pose2d = poseEstimate.pose;
        Translation2d trans = pose2d.getTranslation();
        if (trans.getX() == 0 && trans.getY() == 0) {
            return null;
        }
        return poseEstimate;
    }

    @Override
    public Pose2d getPose() {
        return this.getState().Pose;
    }

    @Override
    public Rotation2d getRotation2d() {
        return m_pigeon2.getRotation2d();
    }

    @Override
    public void stop() {
        setControl(new SwerveRequest.SwerveDriveBrake());
    }

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     *
     * @param pathName      The name of a path located in the "deploy/choreo" directory
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(String pathName, boolean resetPosition) {
        return followChoreoPath(Choreo.getTrajectory(pathName), resetPosition);
    }

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     *
     * @param trajectory    The Choreo trajectory to follow.
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(ChoreoTrajectory trajectory, boolean resetPosition) {
        List<Command> commands = new ArrayList<>();

        if (resetPosition) {
            commands.add(runOnce(() -> {
                seedFieldRelative(DriverStationUtil.isRed() ? trajectory.getFlippedInitialPose() : trajectory.getInitialPose());
            }));
        }
        commands.add(choreoSwerveCommand(trajectory));
        return CommandsUtil.sequence(commands);
    }

    // This is a helper method that creates a command that makes the robot follow a Choreo path
    private Command choreoSwerveCommand(ChoreoTrajectory trajectory) {
        return Choreo.choreoSwerveCommand(
                trajectory,
                this::getPose,
                choreoX,
                choreoY,
                choreoRotation,
                (ChassisSpeeds speeds) -> setControl(
                        autoRequest.withSpeeds(speeds)
                ),
                DriverStationUtil::isRed,
                this
        );
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose,
                this::seedFieldRelative,
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                DriverStationUtil::isRed,
                this);
    }

    public static CommandSwerveDrivetrain getInstance() {
        return TunerConstants.DriveTrain;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Drive/Heading",
                () -> getRotation2d().getDegrees(),
                (DoubleConsumer) null);
    }
}
package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class Telemetry
{
    /// Current Telemetry Setting
    public static TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.HIGH;
    /// Telemetry Defaults
    public static final String telemetryPath = "SmartDashboard/Telemetry/"; // Make access public for other telemetry.
    public static final String smartDashboardPath = "Telemetry/";
    public static final NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable(telemetryPath);

    /// Publishers
    public static class Publishers
    {
        /// Robot Publishers
        public static class Robot
        {
            /// Telemetry Paths
            private static final NetworkTable robotTable = telemetryTable.getSubTable("Robot Telemetry");
            private static final String smartDashboardRobotPath = smartDashboardPath + "Robot Telemetry/";
            // Input Selection
            public static final SmartDashboardPublisher inputPublisher = new SmartDashboardPublisher(smartDashboardRobotPath + "Input Selector");
            public static final StringPublisher inputOverride = robotTable.getSubTable("Input Selector").getStringTopic("selected").publish();

            public static class Vision {
                private static final NetworkTable visionTable = robotTable.getSubTable("Vision");
                private static final List<Pair<Pose3d, StructPublisher<Pose3d>>> cameraPosePublishers = new ArrayList<>();

                public static void publishCameraPose(String camName, Rotation3d camAngle, Translation3d camPose) {
                    cameraPosePublishers.add(Pair.of(new Pose3d(camPose, camAngle), visionTable.getStructTopic(camName + " Pose", Pose3d.struct).publish()));
                }
            }
        }
        /// MapleSim Publishers
        public static class MapleSim
        {
            // Table for maple sim publishers.
            private static final NetworkTable mapleTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/MapleSim");
            // Generic Game Piece Publisher.
            public static final StructArrayPublisher<Pose3d> elementPublisher = mapleTable.getStructArrayTopic("Fuel", Pose3d.struct).publish();
        }
    }

    /// Telemetry Verbosity Settings
    public enum TelemetryVerbosity {
        /// No telemetry data is sent to the dashboard.
        NONE(SwerveDriveTelemetry.TelemetryVerbosity.NONE),
        /// Only basic telemetry data is sent to the dashboard.
        LOW(SwerveDriveTelemetry.TelemetryVerbosity.LOW),
        /// All telemetry data is sent to the dashboard.
        HIGH(SwerveDriveTelemetry.TelemetryVerbosity.HIGH),;

        // Telemetry verbosity for YAGSL at this verbosity level.
        public final SwerveDriveTelemetry.TelemetryVerbosity yagslVerbosity;

        /**
         * Robot Telemetry Options
         *
         * @param yagslVerbosity Verbosity to use for YAGSL at this level.
         */
        TelemetryVerbosity(SwerveDriveTelemetry.TelemetryVerbosity yagslVerbosity) {
            this.yagslVerbosity = yagslVerbosity;
        }
    }

    /// Initializes any need data. Called statically.
    Telemetry() {
        // Add all the default pieces.
        //SimulatedArena.getInstance().resetFieldForAuto();
    }

    /// Updates all of our custom telemetry
    public static void updateTelemetry()
    {
        // Input
        Publishers.Robot.inputPublisher.update();

        if (RobotBase.isSimulation()) {
            // MapleSim
            Publishers.MapleSim.elementPublisher.accept(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        }
    }

    /**
     * Updates camera 3D poses based on the swerve position.
     *
     * @param swervePose the swerve pose.
     */
    public static void updateCameraPoses(Pose3d swervePose) {
        for (Pair<Pose3d, StructPublisher<Pose3d>> pair : Publishers.Robot.Vision.cameraPosePublishers) {
            // Publish the camera pose relative to the robot.
            pair.getSecond().accept(swervePose.plus(new Transform3d(pair.getFirst().getTranslation(), pair.getFirst().getRotation())));
        }
    }

    /**
     * SmartDashboard wrapper to match NT4 Publishers.
     */
    public static class SmartDashboardPublisher
    {
        @Getter
        private Sendable value;
        private Supplier<Sendable> supplier;
        private final String path;

        /**
         * Small SmartDashboard Publisher Wrapper.
         *
         * @param path the telemetry path. Use smartDashboardPath + name.
         */
        public SmartDashboardPublisher(String path)
        {
            this.path = path;

        }

        public void setValue(Sendable value)
        {
            this.value = value;
            SmartDashboard.putData(path, value);
        }

        /**
         * Accepts sendable to be updated later.
         * Also sets when ran.
         *
         * @param supplier the value supplier.
         */
        public void accept(Supplier<Sendable> supplier)
        {
            this.supplier = supplier;
            setValue(supplier.get());
        }

        /**
         * Updates the published value from the current supplier.
         */
        public void update()
        {
            setValue(supplier.get());
        }
    }
}


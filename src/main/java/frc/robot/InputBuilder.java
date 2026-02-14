package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.*;
import lombok.experimental.Accessors;
import swervelib.SwerveInputStream;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.InputBuilder.InputSelections.*;
import static frc.robot.Telemetry.Publishers.Robot.inputOverride;

@SuppressWarnings("unused")
public class InputBuilder
{
    /// Our subsystems and systems to call to from RobotContainer
    private final SwerveSubsystem swerve;
    // Control chooser for dashboard
    private static final SendableChooser<InputSelections> inputSelector = new SendableChooser<>();
    // Control input devices
    private final CommandXboxController driverXbox;

    public InputBuilder(SwerveSubsystem swerveSubsystem) {
        /// Stops the console from being flooded because not all controllers are connected.
        DriverStation.silenceJoystickConnectionWarning(true);
        /// Initialize our subsystem calls
        this.swerve = swerveSubsystem;
        /// Initialize Binding Methods here.
        this.driverXbox = new CommandXboxController(0);
        /// Initialize input publisher
        Telemetry.Publishers.Robot.inputPublisher.accept(() -> inputSelector);

        testing();
    }

    // Control binding type enum
    public enum InputSelections {
        /// Default Input Schema
        SINGLE_XBOX("Single Xbox", false),
        DUAL_XBOX("Dual Xbox"),
        TESTING("Testing", true),

        /**  Define Student Input Selections here  */

        NEW_INPUT("New Student Bindings");

        // BindType Name
        public final String name;
        public final Trigger isMode;


        /** Do NOT Edit Constructors Below */
        /**
         * Constructor for BindingType
         * @param name The name of the control type, used in publishing.
         * @param isDefault whether to make this the default input. There can only be one. No extra handling is done.
         */
        InputSelections(String name, boolean isDefault) {
            this.name = name;
            this.isMode = new Trigger(() -> inputSelector.getSelected() == this);
            if (isDefault) {
                inputSelector.setDefaultOption(name, this);
            } else {
                inputSelector.addOption(name, this);
            }
        }
        /**
         * Constructor for BindingType
         * @param name The name of the control type, used in publishing.
         */
        InputSelections(String name) {
            this(name, false);
        }
    }

    /**
     * A Volatile non important binding for testing purposes.
     */
    public void testing() {
        // Define constants first, like speeds.
        var stream =
                new InputStream().
                        StartingMethod // Dumb constructor to load a different one.
                        .headingXboxDrive(TESTING.isMode, driverXbox)
                        /// Initial Constants
                        .normalRotation(1)
                        .normalTranslation(1)
                        .slowRotation(.4)
                        .slowTranslation(.4)
                        /// Button Bindings
                        .SwerveBindings                 /// SwerveDrive Bindings
                        .withSlowDrive(                 driverXbox.rightBumper())
                        .withToggleCentricity(          driverXbox.back(), true)
                        .withResetSimOdometry(          driverXbox.start())
                        .updateSwerveStream()           // Update our stream after making default drive speed changes.
                        .back().MiscBindings            /// Misc Bindings
                        .withChangeInput(SINGLE_XBOX,   driverXbox.a())
                        .resetField(                    driverXbox.start());
    }

    /// ***** Input Stream Below ***** ///

    @Accessors(fluent = true, chain = true)
    private class InputStream {
        /// Stream Constants
        @Setter private Trigger isMode;
        /// Drive Constants
        @Setter private double deadzone = 0.05;
        @Setter private double slowTranslation = 0.3;
        @Setter private double slowRotation = 0.2;
        @Setter private double normalTranslation = .8;
        @Setter private double normalRotation = .8;
        @Setter private double boostTranslation = 1.0;
        @Setter private double boostRotation = 0.75;
        @Setter @Getter private Supplier<Command> driveCommand; // Default drive command.

        /// Inner Config Namespaces
        public final StartingMethods StartingMethod = new StartingMethods();
        public final SwerveBindings SwerveBindings = new SwerveBindings();
        public final MiscBindings MiscBindings = new MiscBindings();

        private SwerveInputStream swerveInputStream;

        /// Initialize our binding options only when the subsystem is not null.
        InputStream() {
        }

        /// Default Constructor with no drive.
        InputStream(Trigger isMode) {
            this();
            this.isMode = isMode;
        }

        /// Default Basic Drive Constructor
        InputStream(Trigger isMode,
                    DoubleSupplier x,
                    DoubleSupplier y) {
            this(isMode);
            this.swerveInputStream = SwerveInputStream.of(
                    swerve.getSwerveDrive(), x, y) // Make the input stream.
                    .cubeTranslationControllerAxis(true)
                    .cubeRotationControllerAxis(true)
                    .scaleTranslation(normalTranslation)
                    .scaleRotation(normalRotation)
                    .deadband(deadzone)
                    .robotRelative(true)
                    .allianceRelativeControl(false);
            this.driveCommand = () -> swerve.driveFieldOriented(() -> swerveInputStream.get());
            isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(driveCommand().get())));
        }

        /**
         * Basic Control Methods to help get started.
         */
        private class StartingMethods {
            /**
             * Default Xbox Drive Constructor with regular rotation.
             * WARNING: Creates a new stream, DO NOT use inline.
             *
             * @param isMode The trigger telling the stream we are in the Input Selection.
             * @param driverXbox the {@link CommandXboxController} to bind to.
             */
            public InputStream defaultXboxDrive(Trigger isMode, CommandXboxController driverXbox) {
                // Load default drive constructor.
                var xboxDrive = new InputStream(isMode,
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1);
                // Set heading drive.
                xboxDrive.swerveInputStream.withControllerRotationAxis(() -> driverXbox.getRightX() * -1);
                // Return our new stream.
                return xboxDrive;
            }

            /**
             * Default Xbox Drive Constructor with heading rotation.
             * WARNING: Creates a new stream, DO NOT use inline.
             *
             * @param isMode The trigger telling the stream we are in the Input Selection.
             * @param driverXbox the {@link CommandXboxController} to bind to.
             */
            public InputStream headingXboxDrive(Trigger isMode, CommandXboxController driverXbox) {
                // Load default drive constructor.
                var headingDrive = new InputStream(isMode,
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1);
                // Set heading rotation.
                headingDrive.swerveInputStream
                        .cubeRotationControllerAxis(false)
                        .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
                        .headingWhile(true);
                // Return our new stream.
                return headingDrive;
            }
        }

        /**
         * Swerve Drive Button Bindings
         */
        private class SwerveBindings {
            private final boolean isPresent;
            private SwerveBindings() {
                this.isPresent = swerve != null;
            }
            /**
             * Changes inputs to the given input selection.
             */
            public SwerveBindings withToggleCentricity(Trigger toggleCentricity, boolean fieldDefault) {
                // Set our default.
                swerveInputStream.robotRelative(!fieldDefault).allianceRelativeControl(fieldDefault);
                // Toggle when pressed.
                isMode.and(toggleCentricity).toggleOnTrue(Commands.runEnd(
                        () -> swerveInputStream.robotRelative(fieldDefault).allianceRelativeControl(!fieldDefault),
                        () -> swerveInputStream.robotRelative(!fieldDefault).allianceRelativeControl(fieldDefault)));
                return this;
            }

            /**
             * Holding the button slows the drive.
             *
             * @param slowDrive the button to map.
             * @return this, for chaining.
             */
            public SwerveBindings withSlowDrive(Trigger slowDrive) {
                isMode.and(slowDrive)
                        .onTrue(Commands.runOnce(() -> swerveInputStream.scaleTranslation(slowTranslation).scaleRotation(slowRotation)))
                        .onFalse(Commands.runOnce(() -> swerveInputStream.scaleTranslation(normalTranslation).scaleRotation(normalRotation)));
                return this;
            }

            /**
             * Resets the robot odometry. Zeros the gyro. Updates pose to real pose in sim.
             *
             * @param resetOdometry the button to map.
             * @return this, for chaining.
             */
            public SwerveBindings withResetSimOdometry(Trigger resetOdometry) {
                if (!isPresent) {return this;}
                isMode.and(resetOdometry).onTrue(Commands.runOnce(() -> swerve.getSwerveDrive().resetOdometry(swerve.getSwerveDrive().getSimulationDriveTrainPose().get())));
                return this;
            }

            /**
             * Updates the {@link SwerveInputStream} with the latest values.
             *
             * @return this, for chaining.
             */
            public SwerveBindings updateSwerveStream() {
                swerveInputStream
                        .scaleTranslation(normalTranslation)
                        .scaleRotation(normalRotation)
                        .deadband(deadzone);
                return this;
            }

            /**
             * Leaves SwerveBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
                return InputStream.this;
            }
        }


        /**
         * Various Bindings that don't fit into a category yet.
         */
        private class MiscBindings {
            /**
             * Resets the simulated field.
             *
             * @param resetField button to map.
             * @return this, for chaining.
             */
            public MiscBindings resetField(Trigger resetField) {
                isMode.and(() -> RobotBase.isSimulation()).and(resetField).onTrue(
                        Commands.runOnce(() -> SimulatedArena.getInstance().resetFieldForAuto()));
                return this;
            }
            /**
             * Changes inputs to the given input selection.
             *
             * @param bindingType The input to change to.
             * @param changeInput button to map.
             * @return this, for chaining.
             */
            public MiscBindings withChangeInput(InputSelections bindingType, Trigger changeInput) {
                isMode.and(changeInput).onTrue(Commands.runOnce(() -> inputOverride.set(bindingType.name)));
                return this;
            }

            /**
             * Sets a default command for this input selection.
             *
             * @param subsystem the subsystem to apply the default command to.
             * @param defaultCommand the default command to set.
             * @return this, for chaining.
             */
            public MiscBindings withDefaultCommand(SubsystemBase subsystem, Supplier<Command> defaultCommand) {
                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> subsystem.setDefaultCommand(defaultCommand.get())));
                return this;
            }
        }

        /**
         * Leaves MiscBindings going back to the InputStream.
         *
         * @return this InputStream.
         */
        public InputStream back() {
            return InputStream.this;
        }
    }
}
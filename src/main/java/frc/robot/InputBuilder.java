package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.*;
import lombok.experimental.Accessors;
import swervelib.SwerveInputStream;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
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
    }

    /**
     * Method used to construct input streams
     */
    public void configureBindings() {
        // Define constants first, like speeds.
        final InputStream testing =
                new InputStream().
                        startingMethods // Dumb constructor to load a different one.
                        .defaultXboxDrive(TESTING.isMode, driverXbox)
                        /// Initial Constants
                        .normalRotation(.5)
                        .normalTranslation(.5)
                        .slowRotation(.4)
                        .slowTranslation(.4)
                        /// Button Bindings
                        .SwerveBindings                             /// SwerveDrive Bindings
                        .withSlowDrive(                             driverXbox.rightBumper())
                        .withAimWhile(                              driverXbox.rightTrigger(),
                        new Pose2d(new Translation2d(3, 3), Rotation2d.kZero), 
                        new Trigger(() -> false), new Trigger(() -> false))
                        .withToggleCentricity(                      driverXbox.back(), false)
                        .withZeroGyro(                              driverXbox.start())
                        .back().MiscBindings                        /// Misc Bindings
                        .back();
    }

    // Control binding type enum
    public enum InputSelections {
        /// Default Input Schema
        TESTING("Testing", true),
        EASY_DRIVE("Easy drive", false),
        MEDIUM_DRIVE("Medium drive", false),
        FULL_DRIVE("Full drive", false),
        UNLOCKED_DRIVE("Unlocked drive", false),

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

        /**
         * Returns the next {@link InputSelections} value, wrapping around to the first if at the end.
         *
         * @return the next InputSelections enum constant
         */
        public InputSelections getNext() {
            InputSelections[] values = InputSelections.values();
            return values[(ordinal() + 1) % values.length];
        }

        /**
         * Returns the previous {@link InputSelections} value, wrapping around to the last if at the beginning.
         *
         * @return the previous InputSelections enum constant
         */
        public InputSelections getPrevious() {
            InputSelections[] values = InputSelections.values();
            return values[(ordinal() - 1 + values.length) % values.length];
        }
    }

    /// ***** Input Stream Below ***** ///

    @Accessors(fluent = true, chain = true)
    private class InputStream {
        /// Stream Constants
        @Setter
        private Trigger isMode;
        /// Drive Constants
        @Setter
        private double deadzone = 0.05;
        @Setter
        private double slowTranslation = 0.3;
        @Setter
        private double slowRotation = 0.2;
        @Setter
        private double normalTranslation = .8;
        @Setter
        private double normalRotation = .8;
        @Setter
        private double boostTranslation = 1.0;
        @Setter
        private double boostRotation = 0.75;
        @Setter
        @Getter
        private Supplier<Command> driveCommand; // Default drive command.

        /// Inner Config Namespaces
        public final StartingMethods startingMethods = new StartingMethods();
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
        private InputStream(Trigger isMode,
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
             * @param isMode     The trigger telling the stream we are in the Input Selection.
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
             * @param isMode     The trigger telling the stream we are in the Input Selection.
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
            /**
             * Controller Dead Zone
             */
            @Setter
            private double deadzone = 0.01;
            /**
             * Move Speed Scalar when slowing drive speed.
             */
            @Setter
            private double slowTranslation = 0.3;
            /**
             * Rotation Speed Scalar when slowing drive speed.
             */
            @Setter
            private double slowRotation = 0.2;
            /**
             * Move Speed Scalar when driving normally.
             */
            @Setter
            private double normalTranslation = .4;
            /**
             * Rotation Speed Scalar when driving normally.
             */
            @Setter
            private double normalRotation = .75;
            /**
             * Move Speed Scalar when boosting drive speed.
             */
            @Setter
            private double boostTranslation = 1.0;
            /**
             * Rotation Speed Scalar when boosting drive speed.
             */
            @Setter
            private double boostRotation = 0.75;
            /**
             * Drive auto locking tolerance, defaults to 0.01.
             */
            @Setter
            private double autoLockTolerance = 0.01;
            @Setter @Getter
            private Supplier<Command> driveCommand; // Default drive command.

            private final boolean isPresent;

            private SwerveBindings() {
                this.isPresent = swerve != null;
            }

            /**
             * Auto centers the drive wheels when not moving to make the robot harder to move.
             *
             * @return this, for chaining.
             */
            public SwerveBindings withAutoLockDrive() {
                // New trigger to lock the drive when not moving.
                isMode.and(() -> !swerve.isMoving(autoLockTolerance)).whileTrue(Commands.run(swerve::lock));
                return this;
            }

            /**
             * Changes inputs to the given input selection.
             */
            public SwerveBindings withToggleCentricity(Trigger toggleCentricity, boolean fieldDefault) {
                if (!isPresent) {
                    return this;
                }
                // Set our default.
                swerveInputStream.robotRelative(!fieldDefault).allianceRelativeControl(fieldDefault);
                // Toggle when pressed.
                isMode.and(toggleCentricity).toggleOnTrue(Commands.runEnd(
                        () -> swerveInputStream.robotRelative(fieldDefault).allianceRelativeControl(!fieldDefault),
                        () -> swerveInputStream.robotRelative(!fieldDefault).allianceRelativeControl(fieldDefault)));
                return this;
            }

            /**
             * Aims at a pose while held.
             *
             * @param aimWhile      the button to map.
             * @param aimWhilePose  the pose to aim at.
             * @param lookAheadUp   increase look ahead time.
             * @param lookAheadDown decrease look ahead time.
             * @return this, for chaining.
             */
            public SwerveBindings withAimWhile(Trigger aimWhile, Pose2d aimWhilePose, Trigger lookAheadUp, Trigger lookAheadDown) {
                if (!isPresent) {
                    return this;
                }
                // Update Telemetry Continuously
                isMode.and(DriverStation::isEnabled).whileTrue(Commands.run(() -> {
                    SmartDashboard.putBoolean("Aim Data/isLocked", swerveInputStream.aimLock(Degrees.of(1)).getAsBoolean());
                }));
                // Save an adjustable atomic time.
                final int[] lookAheadTime = {0};
                // Add 1s to lookAheadTime
                isMode.and(lookAheadUp).onTrue(Commands.runOnce(() -> {
                    lookAheadTime[0]++;
                    swerveInputStream.aimLookahead(Seconds.of(lookAheadTime[0]));
                }));
                // Remove 1s from lookAheadTime
                isMode.and(lookAheadDown).onTrue(Commands.runOnce(() -> {
                    lookAheadTime[0]--;
                    swerveInputStream.aimLookahead(Seconds.of(lookAheadTime[0]));
                }));
                // Aim while held
                isMode.and(aimWhile).onTrue(Commands.runOnce(() -> {
                    // Update our pose and aim supplier when isMode and aimWhile.
                    swerveInputStream.aim(AllianceFlipUtil.ifShouldFlip(aimWhilePose));
                    swerveInputStream.aimWhile(isMode.and(aimWhile));
                }));
                // Return this for chaining.
                return this;
            }

            /**
             * Holding the button slows the drive.
             *
             * @param slowDrive the button to map.
             * @return this, for chaining.
             */
            public SwerveBindings withSlowDrive(Trigger slowDrive) {
                if (!isPresent) {
                    return this;
                }
                isMode.and(slowDrive)
                        .onTrue(Commands.runOnce(() -> swerveInputStream.scaleTranslation(slowTranslation).scaleRotation(slowRotation)))
                        .onFalse(Commands.runOnce(() -> swerveInputStream.scaleTranslation(normalTranslation).scaleRotation(normalRotation)));
                return this;
            }

            /**
             * Holding the button boosts the drive.
             *
             * @param boostDrive the button to map.
             * @return this, for chaining.
             */
            public SwerveBindings withBoostDrive(Trigger boostDrive) {
                if (!isPresent) {
                    return this;
                }
                isMode.and(boostDrive)
                        .onTrue(Commands.runOnce(() -> swerveInputStream.scaleTranslation(boostTranslation).scaleRotation(boostRotation)))
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
                if (!isPresent) {
                    return this;
                }
                isMode.and(resetOdometry).onTrue(Commands.runOnce(() -> swerve.getSwerveDrive().resetOdometry(swerve.getSwerveDrive().getSimulationDriveTrainPose().get())));
                return this;
            }

            /**
             * Resets the robot odometry. Zeros the gyro. Updates pose to real pose in sim.
             *
             * @param zeroGyro the button to map.
             * @return this, for chaining.
             */
            public SwerveBindings withZeroGyro(Trigger zeroGyro) {
                if (!isPresent) { return this; }
                isMode.and(zeroGyro).onTrue(Commands.runOnce(() -> {
                    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
                        // If red zero backwards
                        swerve.zeroGyro();
                        swerve.getSwerveDrive().resetOdometry(
                            new Pose2d(swerve.getSwerveDrive().getPose().getTranslation(),
                            Rotation2d.fromDegrees(180)));
                    } else {
                        swerve.getSwerveDrive().zeroGyro();
                    }
            }));
                return this;
            }

            /**
             * Sets a default command for this subsystem.
             *
             * @param defaultCommand the default command to set.
             * @return this, for chaining.
             */
            public SwerveBindings withDefaultCommand(Supplier<Command> defaultCommand) {
                if (!isPresent) {
                    return this;
                }
                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(defaultCommand.get())));
                return this;
            }

            /**
             * Leaves SwerveBindings going back to the InputStream.
             *
             * @return this InputStream.
             */
            public InputStream back() {
                if (!isPresent) {return InputStream.this;}
                swerveInputStream
                        .scaleTranslation(normalTranslation)
                        .scaleRotation(normalRotation)
                        .deadband(deadzone);
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
                isMode.and(RobotBase::isSimulation).and(resetField).onTrue(
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
                isMode.and(changeInput.debounce(0.2)).onTrue(Commands.runOnce(() -> inputOverride.set(bindingType.name)));
                return this;
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
}
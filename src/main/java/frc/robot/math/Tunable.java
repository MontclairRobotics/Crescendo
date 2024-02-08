package frc.robot.math;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A value that is tunable via NetworkTables (and thus via SmartDashboard,
 * Shuffleboard, etc)
 * 
 * When value is changed in the NetworkTable, any {@link Consumer} that
 * registers for updates
 * with {@link #whenUpdate(Consumer)} will be called.
 * 
 * This only works in testing mode
 */
public abstract class Tunable<T> implements Sendable {
    // The value
    T value;

    // List of consumers that get updated when values changes
    ArrayList<Consumer<T>> updators = new ArrayList<>();

    // The entry in the network table
    public final GenericEntry entry;

    // Name of the entry
    final String name;

    // Sets the value in Network tables
    // Overwritten by type specific classes of Tunable
    protected abstract void setNT(T value);

    // Reads value from network tables
    // Overwritten by type specific classes of Tunable
    protected abstract T getFromNT();

    /**
     * Protected base constructor
     * Only type specific constructors are public
     * 
     * @param value - Default value
     * @param name  - Name
     */
    protected Tunable(T value, String name) {
        this.value = value;
        this.name = name;

        // Get NetworkTable entry
        NetworkTable nt = NetworkTableInstance.getDefault().getTable("Tuning");
        entry = nt.getTopic(name).getGenericEntry();

        // Set default value
        setNT(value);

        // Listen for changes to NetworkTable entry
        nt.addListener(name, EnumSet.of(Kind.kValueAll), (table, key, event) -> {
            DriverStation.MatchType matchType = DriverStation.getMatchType();

            // Make sure we are in testing mode!!!!
            if (matchType == DriverStation.MatchType.None
                    || matchType == DriverStation.MatchType.Practice
                    || DriverStation.isTest()) {

                // Get new value
                this.value = getFromNT();

                // Call each consumer with the new value
                for (Consumer<T> updator : updators) {
                    updator.accept(this.value);
                }
            }
        });
    }

    /**
     * Creates a Tunable for doubles
     * 
     * @param value Default value
     * @param name Name
     * @return The Tunable
    */
    public static Tunable<Double> of(double value, String name) {
        return new Tunable<Double>(value, name) {
            @Override
            protected Double getFromNT() {
                return entry.getDouble(value);
            }

            @Override
            protected void setNT(Double value) {
                entry.setDouble(value);
            }

            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("value", this::get, this::setNT);
            }
        };
    }

    /**
     * Creates a Tunable for boolean
     * 
     * @param value Default value
     * @param name Name
     * @return The Tunable
    */
    public static Tunable<Boolean> of(boolean value, String name) {
        return new Tunable<Boolean>(value, name) {
            @Override
            protected Boolean getFromNT() {
                return entry.getBoolean(value);
            }

            @Override
            protected void setNT(Boolean value) {
                entry.setBoolean(value);
            }

            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("value", this::get, this::setNT);
            }
        };
    }

    /**
     * Register a consumer to receive updates whenever the Tunable's value changes
     * 
     * @param updator The consumer
     * @return The Tunable
     */
    public Tunable<T> whenUpdate(Consumer<T> updator) {
        updators.add(updator);
        return this;
    }

    /**
     * Returns the current value
     */
    public T get() {
        return value;
    }
}

package frc.lib;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Handle for an object whose value is modifiable through NetworkTables.
 *
 * @param <R> The resulting object type.
 */
public class Tunable<R> implements Supplier<Optional<R>> {

    /**
     * A function that retrieves the NetworkTables value as a double.
     */
    private final DoubleSupplier value;

    /**
     * A function that retrieves if the NetworkTables value should be used.
     */
    private final BooleanSupplier isEnabled;

    /**
     * A function that converts from a double value to the resulting object type.
     */
    private final Function<Double, R> constructor;

    /**
     * Creates a handle for an object whose value is modified through NetworkTables.
     * @param name The name for this object, as seen through NetworkTables.
     * @param constructor A function that converts from a double value to the resulting object type.
     * @param defaultValue The default value to display in NetworkTables.
     */
    public Tunable(String name, Function<Double, R> constructor, double defaultValue) {
        this.value = createValue(name, defaultValue);
        this.isEnabled = createIsEnabled(name, false);
        this.constructor = constructor;
    }

    /**
     * Creates a handle for an object whose value is modified through NetworkTables.
     * @param name The name for this object, as seen through NetworkTables.
     * @param constructor A function that converts from a double value to the resulting object type.
     */
    public Tunable(String name, Function<Double, R> constructor) {
        this(name, constructor, 0.0);
    }

    /**
     * Creates a unit-safe handle for a value modified through NetworkTables.
     * @param name The name for the value, as seen through NetworkTables.
     * @param Unit The unit of the value.
     * @return A unit-safe handle for a value modified through NetworkTables.
     * @param <U> The unit of the value.
     */
    public static <U extends Unit> Tunable<Measure<U>> ofUnit(String name, U Unit) {
        String nameWithUnit = String.format("%s (%s)", name, Unit.name());
        return new Tunable<>(nameWithUnit, d -> (Measure<U>) Unit.of(d));
    }

    /**
     * Sets up the function to retrieve the value from NetworkTables.
     * @param name The name for the value, as seen through NetworkTables.
     * @param defaultValue The default value to display in NetworkTables.
     * @return the function to retrieve the value from NetworkTables.
     */
    private static DoubleSupplier createValue(String name, double defaultValue) {
        DoubleTopic topic = NetworkTableInstance.getDefault().getDoubleTopic(name + "/Value");
        // Create a publisher for the topic and set the default value to
        // ensure that the publisher is initially shown in NetworkTables
        topic.publish().set(defaultValue);
        return topic.subscribe(defaultValue);
    }

    /**
     * Sets up the function to retrieve if the NetworkTables value should be used.
     * @param name The name for the value, as seen through NetworkTables.
     * @param defaultValue The default enable state.
     * @return the function to retrieve if the NetworkTables value should be used.
     */
    private static BooleanSupplier createIsEnabled(String name, boolean defaultValue) {
        BooleanTopic topic = NetworkTableInstance.getDefault().getBooleanTopic(name + "/IsEnabled");
        // Create a publisher for the topic and set the default value to
        // ensure that the publisher is initially shown in NetworkTables
        topic.publish().set(defaultValue);
        return topic.subscribe(defaultValue);
    }

    @Override
    public Optional<R> get() {
        if (isEnabled.getAsBoolean()) {
            return Optional.of(constructor.apply(value.getAsDouble()));
        }
        return Optional.empty();
    }

}

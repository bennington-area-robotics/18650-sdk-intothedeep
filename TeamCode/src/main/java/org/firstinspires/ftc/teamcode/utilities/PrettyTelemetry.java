package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * A lightweight, pre-configured wrapper for the FTC Telemetry system that only supports supplier-based updating,
 * with no clearing. This wrapper improves readability by incorporating automatic rounding and HTML formatting.
 *
 * <h5>Features:</h5>
 * <ul>
 * <li>Automatic rounding of numerical values to a specified precision.</li>
 * <li>HTML-based formatting for better visual clarity.</li>
 * <li>Supports telemetry for both the driver station and an optional FTC Dashboard instance.</li>
 * <li>Simplifies telemetry logging by maintaining a list of captions and value suppliers.</li>
 * </ul>
 */

public class PrettyTelemetry {
    private double roundingPlaces;

    private final Telemetry telemetry;
    private Telemetry dashTelemetry;

    private final List<String> dashValueCaptions = new ArrayList<>();
    private final List<Func<?>> dashValueProducers = new ArrayList<>();

    /**
     * Constructs a `PrettyTelemetry` instance using the provided `Telemetry` object.
     *
     * @param telemetry The FTC `Telemetry` instance.
     */
    public PrettyTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
        this.roundingPlaces = 3;
        telemetry.setAutoClear(false);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator("");
    }

    /**
     * Constructs a PrettyTelemetry instance with support for both driver station telemetry and FTC Dashboard telemetry.
     *
     * @param opmodeTelemetry       The FTC Telemetry instance for the driver station.
     * @param ftcDashboardTelemetry The FTC Telemetry instance for the dashboard.
     */
    public PrettyTelemetry(Telemetry opmodeTelemetry, Telemetry ftcDashboardTelemetry){
        this.telemetry = opmodeTelemetry;
        this.dashTelemetry = ftcDashboardTelemetry;
        this.roundingPlaces = 3;
        telemetry.setAutoClear(false);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator("");
    }

    /**
     * Checks if the telemetry instance has an associated FTC Dashboard telemetry.
     *
     * @return `true` if FTC Dashboard telemetry is available, otherwise `false`.
     */
    public boolean hasDashboard(){
        return dashTelemetry != null;
    }

    /**
     * Adds a telemetry data entry to the FTC Dashboard.
     *
     * @param caption       The label for the data entry.
     * @param valueProducer A function that supplies the value dynamically.
     * @param <T>           The type of value being provided.
     */
    public <T> void addDataToDashboard(String caption, Func<T> valueProducer){
        dashValueCaptions.add(caption);
        dashValueProducers.add(wrapFunc(valueProducer));
    }

    /**
     * Updates the telemetry outputs for both the driver station and, if available, the FTC Dashboard.
     */
    public void update(){
        telemetry.update();

        if(hasDashboard()){
            for (int i = 0; i < dashValueCaptions.size(); i++) {
                dashTelemetry.addData(dashValueCaptions.get(i), dashValueProducers.get(i).value());
            }

            dashTelemetry.update();
        }
    }

    /**
     * Retrieves the number of decimal places used for rounding numerical values.
     *
     * @return The current rounding precision.
     */
    public double getRoundingPlaces() {
        return roundingPlaces;
    }

    /**
     * Sets the number of decimal places to which numerical values should be rounded.
     *
     * @param roundingPlaces The desired precision for rounding.
     */
    public void setRoundingPlaces(double roundingPlaces) {
        this.roundingPlaces = roundingPlaces;
    }

    /**
     * Represents a formatted telemetry line that allows for structured logging.
     */
    public static class Line {
        Telemetry.Line line;
        private Line(Telemetry.Line line){
            this.line = line;
        }

        /**
         * Adds a formatted telemetry data entry to the current line.
         *
         * @param caption       The label for the data entry.
         * @param valueProducer A function that supplies the value dynamically.
         * @param <T>           The type of value being provided.
         * @return A reference to the newly created `Item`.
         */
        public <T> Item addData(String caption, Func<T> valueProducer){
            return new Item(line.addData("<br>- " + caption, wrapFunc(valueProducer)));
        }
    }

    /**
     * Represents an individual telemetry data item within a line.
     */
    public static class Item {
        Telemetry.Item item;
        private Item(Telemetry.Item item){
            this.item = item;
        }

        /**
         * Adds additional telemetry data to the current item.
         *
         * @param caption       The label for the additional data entry.
         * @param valueProducer A function that supplies the value dynamically.
         * @param <T>           The type of value being provided.
         * @return A reference to the newly created `Item`.
         */
        public <T> Item addData(String caption, Func<T> valueProducer){
            return new Item(item.addData("<br>- " + caption, wrapFunc(valueProducer)));
        }
    }

    /**
     * Adds a new formatted telemetry line with a bold caption.
     *
     * @param caption The title of the telemetry line.
     * @return A `Line` object representing the newly added telemetry line.
     */
    public PrettyTelemetry.Line addLine(String caption){
        return new Line(telemetry.addLine("<br><b>" + caption + "</b>"));
    }


    /**
     * Adds a new empty telemetry line.
     *
     * @return A `Line` object representing the newly added empty telemetry line.
     */
    public PrettyTelemetry.Line addLine(){
        return new Line(telemetry.addLine());
    }

    /**
     * Adds a formatted telemetry data entry to the telemetry system.
     *
     * @param caption       The label for the data entry.
     * @param valueProducer A function that supplies the value dynamically.
     * @param <T>           The type of value being provided.
     * @return A reference to the newly created `Item`.
     */
    public <T> Item addData(String caption, Func<T> valueProducer){
        return new Item(telemetry.addData("<br>- " + caption, wrapFunc(valueProducer)));
    }

    /**
     * Wraps a value-producing function to apply rounding where applicable.
     *
     * @param valueProducer The function that supplies the value.
     * @param <T>           The type of value being provided.
     * @return A wrapped `Func<?>` that applies rounding to numerical values.
     */
    public static <T> Func<?> wrapFunc(Func<T> valueProducer){
        T value = valueProducer.value();
        if(value instanceof Double){
	        //noinspection unchecked
	        Func<Double> doubleProducer = (Func<Double>) valueProducer;
            return () -> roundToPrecision(doubleProducer.value(), 3);
        } else if (value instanceof Float) {
            //noinspection unchecked
            Func<Float> floatProducer = (Func<Float>) valueProducer;
            return () -> roundToPrecision(floatProducer.value(), 3);
        }else{
            return valueProducer;
        }
    }

    /**
     * Rounds a numerical value to the specified number of decimal places.
     *
     * @param value         The value to round.
     * @param decimalPlaces The number of decimal places to retain.
     * @return The rounded value.
     * @throws IllegalArgumentException if decimalPlaces is negative.
     */
    public static double roundToPrecision(double value, int decimalPlaces) {
        if (decimalPlaces < 0) throw new IllegalArgumentException("decimalPlaces must be non-negative");

        double factor = Math.pow(10, decimalPlaces);
        return Math.round(value * factor) / factor;
    }
}

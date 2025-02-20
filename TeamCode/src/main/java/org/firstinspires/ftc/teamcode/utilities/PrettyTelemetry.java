package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Light pre-configured wrapper for Telemetry which only supports supplier-based updating, with no clearing.
 * This wrapper builds in some features to make the output more readable. This includes automatic rounding and HTML formatting.
 */
public class PrettyTelemetry {
    private double roundingPlaces;

    private final Telemetry telemetry;
    private Telemetry dashTelemetry;

    private final List<String> dashValueCaptions = new ArrayList<>();
    private final List<Func<?>> dashValueProducers = new ArrayList<>();

    public PrettyTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
        this.roundingPlaces = 3;
        telemetry.setAutoClear(false);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator("");
    }

    public PrettyTelemetry(Telemetry opmodeTelemetry, Telemetry ftcDashboardTelemetry){
        this.telemetry = opmodeTelemetry;
        this.dashTelemetry = ftcDashboardTelemetry;
        this.roundingPlaces = 3;
        telemetry.setAutoClear(false);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator("");
    }

    public boolean hasDashboard(){
        return dashTelemetry != null;
    }

    public <T> void addDataToDashboard(String caption, Func<T> valueProducer){
        dashValueCaptions.add(caption);
        dashValueProducers.add(wrapFunc(valueProducer));
    }

    public void update(){
        telemetry.update();

        if(dashTelemetry != null){
            for (int i = 0; i < dashValueCaptions.size(); i++) {
                dashTelemetry.addData(dashValueCaptions.get(i), dashValueProducers.get(i));
            }

            dashTelemetry.update();
        }
    }

    public double getRoundingPlaces() {
        return roundingPlaces;
    }

    public void setRoundingPlaces(double roundingPlaces) {
        this.roundingPlaces = roundingPlaces;
    }

    public static class Line {
        Telemetry.Line line;
        private Line(Telemetry.Line line){
            this.line = line;
        }

        public <T> Item addData(String caption, Func<T> valueProducer){
            return new Item(line.addData("<br>- " + caption, wrapFunc(valueProducer)));
        }
    }

    public static class Item {
        Telemetry.Item item;
        private Item(Telemetry.Item item){
            this.item = item;
        }

        public <T> Item addData(String caption, Func<T> valueProducer){
            return new Item(item.addData("<br>- " + caption, wrapFunc(valueProducer)));
        }
    }

    public PrettyTelemetry.Line addLine(String caption){
        return new Line(telemetry.addLine("<br><b>" + caption + "</b>"));
    }

    public PrettyTelemetry.Line addLine(){
        return new Line(telemetry.addLine());
    }

    public <T> Item addData(String caption, Func<T> valueProducer){
        return new Item(telemetry.addData("<br>- " + caption, wrapFunc(valueProducer)));
    }

    /** @noinspection unchecked*/
    public static <T> Func<?> wrapFunc(Func<T> valueProducer){
        T value = valueProducer.value();
        if(value instanceof Double){
            Func<Double> doubleProducer = (Func<Double>) valueProducer;
            return () -> roundToPrecision(doubleProducer.value(), 3);
        } else if (value instanceof Float) {
            Func<Float> floatProducer = (Func<Float>) valueProducer;
            return () -> roundToPrecision(floatProducer.value(), 3);
        }else{
            return valueProducer;
        }
    }

    public static double roundToPrecision(double value, int decimalPlaces) {
        if (decimalPlaces < 0) throw new IllegalArgumentException("decimalPlaces must be non-negative");

        double factor = Math.pow(10, decimalPlaces);
        return Math.round(value * factor) / factor;
    }
}

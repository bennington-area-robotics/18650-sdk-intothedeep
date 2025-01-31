package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;

/**
 * Light pre-configured wrapper for Telemetry which only supports supplier-based updating, with no clearing.
 */
public class PrettyTelemetry {
    private final Telemetry telemetry;
    public PrettyTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
        telemetry.setAutoClear(false);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator("");
    }

    public void update(){
        telemetry.update();
    }

    public static class Line {
        Telemetry.Line line;
        private Line(Telemetry.Line line){
            this.line = line;
        }

        public <T> Item addData(String caption, Supplier<T> valueProducer){
            return new Item(line.addData("<br>- " + caption, valueProducer));
        }
    }

    public static class Item {
        Telemetry.Item item;
        private Item(Telemetry.Item item){
            this.item = item;
        }

        public <T> Item addData(String caption, Supplier<T> valueProducer){
            return new Item(item.addData("<br>- " + caption, valueProducer));
        }
    }

    public PrettyTelemetry.Line addLine(String caption){
        return new Line(telemetry.addLine("<br><b>" + caption + "</b>"));
    }

    public PrettyTelemetry.Line addLine(){
        return new Line(telemetry.addLine());
    }

    public <T> Item addData(String caption, Supplier<T> valueProducer){
        return new Item(telemetry.addData("<br>- " + caption, valueProducer));
    }
}

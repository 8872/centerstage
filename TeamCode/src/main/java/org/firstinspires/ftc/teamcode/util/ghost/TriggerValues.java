package org.firstinspires.ftc.teamcode.util.ghost;

import java.util.ArrayList;
import java.util.Arrays;

public class TriggerValues extends ControllerValues<Double> {
    public final static String leftTrigger = "lt";
    public final static String rightTrigger = "rt";

    public TriggerValues() {
        super(0.0,
                new ArrayList<>(Arrays.asList(new Double[]{0.0, 0.0})),
                new ArrayList<>(Arrays.asList(new String[]{leftTrigger, rightTrigger}))
        );
    }
}

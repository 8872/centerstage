package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedList;
import java.util.Queue;

public class HighestSampleFilter {
    private final Queue<Double> queue;
    private final int size;

    public HighestSampleFilter(int size) {
        this.size = size;
        queue = new LinkedList<>();
    }

    public double calculate(double input) {
        queue.add(input);
        if (queue.size() > 5) {
            queue.poll();
        }
        return queue.stream().max(Double::compare).get();
    }

    public Queue<Double> getQueue() {
        return queue;
    }
}

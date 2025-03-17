package org.firstinspires.ftc.teamcode.hardware.filters;

import java.util.ArrayDeque;
import java.util.Collections;
import java.util.PriorityQueue;

/**
 * @noinspection DataFlowIssue
 */
public class RollingMedian implements DataFilter {
	private final int maxSize;
	private final PriorityQueue<Double> minHeap = new PriorityQueue<>(); // Larger half
	private final PriorityQueue<Double> maxHeap = new PriorityQueue<>(Collections.reverseOrder()); // Smaller half
	private final ArrayDeque<Double> window = new ArrayDeque<>(); // To track old values

	public RollingMedian(int maxSize){
		this.maxSize = maxSize;
	}

	public double compute(double num){
		if(maxHeap.isEmpty() || num <= maxHeap.peek()){
			maxHeap.offer(num);
		}else{
			minHeap.offer(num);
		}

		// Balance the heaps
		if(maxHeap.size() > minHeap.size() + 1){
			minHeap.offer(maxHeap.poll());
		}else if(minHeap.size() > maxHeap.size()){
			maxHeap.offer(minHeap.poll());
		}

		window.addLast(num);

		// Remove oldest element if exceeding maxSize
		if(window.size() > maxSize){
			double removed = window.pollFirst();
			if(maxHeap.contains(removed)){
				maxHeap.remove(removed);
			}else{
				minHeap.remove(removed);
			}
			reBalance();
		}

		return getMedian();
	}

	private void reBalance(){
		while(maxHeap.size() > minHeap.size() + 1) minHeap.offer(maxHeap.poll());
		while(minHeap.size() > maxHeap.size()) maxHeap.offer(minHeap.poll());
	}

	private double getMedian(){
		return maxHeap.size() == minHeap.size()
				? (maxHeap.peek() + minHeap.peek()) / 2.0
				: maxHeap.peek();
	}
}

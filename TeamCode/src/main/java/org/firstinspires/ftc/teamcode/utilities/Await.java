package org.firstinspires.ftc.teamcode.utilities;

import java.util.function.Supplier;

public class Await {
	private static final Object lock = new Object();

	/**
	 * Blocks the current thread until the given condition evaluates to true.
	 *
	 * @param condition The condition to wait for.
	 */
	public static void condition(Supplier<Boolean> condition) {
		synchronized (lock) {
			while (!condition.get()) {
				try {
					lock.wait();
				} catch (InterruptedException e) {
					Thread.currentThread().interrupt();
					break;
				}
			}
		}
	}

	/**
	 * Notifies all waiting threads that the condition may have changed.
	 * Should be called when the condition becomes true.
	 */
	public static void notifyChange() {
		synchronized (lock) {
			lock.notifyAll(); // wakes up all waiting threads
		}
	}
}

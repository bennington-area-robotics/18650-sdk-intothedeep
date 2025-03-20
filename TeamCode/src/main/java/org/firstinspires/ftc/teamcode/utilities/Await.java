package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.core.BasicOpModeCore;

import java.util.function.Supplier;

/** @noinspection BusyWait*/
public class Await {

	/**
	 * Blocks the current thread until the given condition evaluates to true, polling at a fixed interval.
	 *
	 * @param condition   The condition to wait for.
	 * @param pollingRate The interval (in milliseconds) to check the condition.
	 */
	public static void condition(Supplier<Boolean> condition, long pollingRate) {
		while (!condition.get() && !BasicOpModeCore.getInstance().isStopRequested()) {
			try {
				Thread.sleep(pollingRate);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				break;
			}
		}
	}

	/**
	 * Blocks the current thread until the notify method is called on the given Notifier.
	 *
	 * @param notifier The Notifier instance to wait for the notify call.
	 */
	public static void notification(Notifier notifier) {
		//noinspection SynchronizationOnLocalVariableOrMethodParameter
		synchronized (notifier) {
			try {
				notifier.resetNotification();
				// this is to make sure spurious wake-ups don't cause the thread to exit prematurely
				while (!notifier.isNotified() && !BasicOpModeCore.getInstance().isStopRequested()) {
					notifier.wait();
				}
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
			}
		}
	}
}

package org.firstinspires.ftc.teamcode.utilities;

/**
 * A simple object intended to notify waiting threads to wake up. Uses a custom method <code>notifyWaitingThreads()</code> to avoid spurious wake-ups.
 * Using <code>notify()</code> will not wake waiting threads.
 */
public class Notifier {
	private boolean notified = false;

	/**
	 * Notifies the waiting thread.
	 */
	public synchronized void notifyWaitingThreads() {
		this.notified = true;
		this.notify();
	}

	/**
	 * Checks whether the waiting thread has been notified.
	 *
	 * @return true if notified, false otherwise.
	 */
	public synchronized boolean isNotified() {
		return notified;
	}

	/**
	 * Resets the notification status.
	 */
	public synchronized void resetNotification() {
		this.notified = false;
	}
}

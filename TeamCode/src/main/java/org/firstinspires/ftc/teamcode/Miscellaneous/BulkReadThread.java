package org.firstinspires.ftc.teamcode.Miscellaneous;

/**
 * A thread that continuously polls bulk data from the Expansion Hub.
 * It is extremely important to make sure that no other thread performs any sort of bus transaction
 * to the Expansion Hub while this thread is running.
 */

public class BulkReadThread implements Runnable {
    public static volatile boolean kill = true;

    @Override
    public void run() {
        kill = false;
        while (!kill && !Thread.currentThread().isInterrupted()) {
            GlobalBulkRead.updateBulkData();
        }
        kill = true;
    }
}

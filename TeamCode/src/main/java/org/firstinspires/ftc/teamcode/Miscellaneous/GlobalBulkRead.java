package org.firstinspires.ftc.teamcode.Miscellaneous;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

/**
 * A class to handle Bulk Reads. It provides asynchronous control via volatile variables
 * In order to prevent thread collisions, it is extremely important to make sure that no other thread performs any sort of bus transaction
 * to the Expansion Hub while this is used.
 * Might not be the best way to do this. Maybe use Reentrant Read-Write lock?
 */
public class GlobalBulkRead {

    public static volatile RevBulkData bulkData1;
    public static volatile RevBulkData bulkData2;

    private static ExpansionHubEx expansionHub1, expansionHub2;

    public static void updateBulkData() {
        bulkData1 = expansionHub1.getBulkInputData();
        bulkData2 = expansionHub2.getBulkInputData();
    }

    public static void setExpansionHubs(ExpansionHubEx e1, ExpansionHubEx e2) {
        expansionHub1 = e1;
        expansionHub2 = e2;
    }

    public static void resetBulkData() {
        bulkData1 = null;
        bulkData2 = null;
    }

}

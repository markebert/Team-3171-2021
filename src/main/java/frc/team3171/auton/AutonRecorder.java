package frc.team3171.auton;

// Java Imports
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.ConcurrentLinkedQueue;

import edu.wpi.first.wpilibj.Timer;

/**
 * @author Mark Ebert
 * @author Charles Fee
 * @author Elijah Hoda
 */
public class AutonRecorder {

    private final ConcurrentLinkedQueue<AutonRecorderData> autonQueue;
    // private volatile AutonRecorderData lastData = null;

    /**
     * Constructor
     */
    public AutonRecorder() {
        autonQueue = new ConcurrentLinkedQueue<>();
    }

    /**
     * 
     */
    public void addNewData(final AutonRecorderData newData) {
        if (newData != null) {
            autonQueue.add(newData);
            /*
             * if (lastData == null) { autonQueue.add(newData); lastData = newData; } else
             * if (lastData.isChanged(newData)) { autonQueue.add(newData); lastData =
             * newData; }
             */
        }
    }

    /**
     * Clears whatever the AutonRecorder currently has saved.
     */
    public void clear() {
        autonQueue.clear();
    }

    /**
     * Saves the current data collected by the auton recorder to the specified file
     * path and clears the AutonRecorder.
     * 
     * @param autonFileName The path to the file to save the auton data to.
     */
    public void saveToFile(String autonFileName) {
        try {
            File autonFile = new File(String.format("/home/lvuser/%s.txt", autonFileName));
            if (autonFile.exists()) {
                autonFile.delete();
            }
            autonFile.createNewFile();
            BufferedWriter writer = new BufferedWriter(new FileWriter(autonFile));
            AutonRecorderData startData = new AutonRecorderData();
            startData.setFPGATimestamp(0);
            writer.write(startData.toString());
            writer.flush();
            // CHANGE THE NAME OF THE TEXT FILE BASED OFF OF WHAT AUTON YOU ARE WRITING FOR
            AutonRecorderData autonData = autonQueue.poll();
            while (autonData != null) {
                writer.write(autonData.toString());
                writer.flush();
                autonData = autonQueue.poll();
                Timer.delay(.001);
            }
            // Write an auton end
            //AutonRecorderData endData = new AutonRecorderData();
            //endData.setFPGATimestamp(autonData.getFPGATimestamp() + .1);
            //writer.write(endData.toString());
            // Flush the data to the file
            writer.flush();
            writer.close();
            System.out.println("Auton Successfully Saved!");
        } catch (IOException e) {
            e.printStackTrace();
        }
        autonQueue.clear();
    }

    /**
     * 
     * @param autonFileName
     * @return
     */
    public static void loadFromFile(ConcurrentLinkedQueue<AutonRecorderData> autonPlaybackQueue, String autonFileName) {
        autonPlaybackQueue.clear();
        try {
            BufferedReader reader = new BufferedReader(
                    new FileReader(String.format("/home/lvuser/%s.txt", autonFileName)));
            String line = reader.readLine();
            while (line != null) {
                autonPlaybackQueue.add(AutonRecorderData.fromString(line));
                line = reader.readLine();
            }
            reader.close();
            System.out.println("Auton Successfully Loaded!");
        } catch (IOException e) {
            System.err.printf("The Auton File %s.txt could not be found!\n", autonFileName);
        }
    }

}

package frc.util;

import edu.wpi.first.wpilibj.Timer;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class RepeatabilityLogger {

    private final BufferedWriter writer;
    private final Timer timer = new Timer();

    public RepeatabilityLogger(String filename) {
        try {
            File file = new File("/U/" + filename);
            writer = new BufferedWriter(new FileWriter(file));
            writer.write("time_sec,x_m,y_m,theta_deg,distance_to_goal_m,angle_error_deg\n");
            timer.start();
        } catch (IOException e) {
            throw new RuntimeException("Failed to open log file", e);
        }
    }

    public void log(double x,
                    double y,
                    double thetaDeg,
                    double distanceToGoal,
                    double angleErrorDeg) {

        try {
            writer.write(
                    timer.get() + "," +
                    x + "," +
                    y + "," +
                    thetaDeg + "," +
                    distanceToGoal + "," +
                    angleErrorDeg + "\n"
            );
            writer.flush();  // For testing reliability. Batch later if desired.
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void close() {
        try {
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
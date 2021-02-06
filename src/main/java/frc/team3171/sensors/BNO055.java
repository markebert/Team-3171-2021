package frc.team3171.sensors;

// Java Imports
import java.util.function.DoubleSupplier;

// FRC Imports
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * @author Mark Ebert
 */
public class BNO055 implements DoubleSupplier {

    private SerialPort serial;

    private volatile double x = 0, y = 0, z = 0, resetX = 0;
    private volatile double lastDataRecievedTimestamp = 0;
    private volatile boolean possibleError = false;

    private static final int BAUD_RATE = 115200, TIMEOUT = 1;

    public BNO055(SerialPort.Port port) {
        serial = new SerialPort(BAUD_RATE, port);

        new Thread(() -> {
            String gyroString = "";
            while (true) {
                try {
                    while (serial.getBytesReceived() > 0) {
                        char c = (char) serial.read(1)[0];
                        if (c == '\n') {
                            for (String s : gyroString.split(",")) {
                                String[] data = s.split(":");
                                if (data.length == 2) {
                                    try {
                                        switch (data[0]) {
                                        case "X":
                                            x = Normalize_Gryo_Value(Double.valueOf(data[1]));
                                            break;
                                        case "Y":
                                            y = Double.valueOf(data[1]);
                                            break;
                                        case "Z":
                                            z = Double.valueOf(data[1]);
                                            break;
                                        default:
                                            break;
                                        }
                                        // Valid data
                                        possibleError = false;
                                    } catch (Exception e) {
                                        // Do Nothing
                                        possibleError = true;
                                    }
                                }
                            }
                            if ((x != 0.0) || (y != 0.0) || (z != 0.0)) {
                                lastDataRecievedTimestamp = Timer.getFPGATimestamp();
                            }
                            gyroString = "";
                        } else {
                            gyroString += c;
                        }
                    }
                    if (Timer.getFPGATimestamp() > (lastDataRecievedTimestamp + TIMEOUT)) {
                        possibleError = true;
                        x = 0;
                        y = 0;
                        z = 0;
                    }
                } catch (Exception e) {
                    possibleError = true;
                    // Reset gyro values
                    x = 0;
                    y = 0;
                    z = 0;
                }
                Timer.delay(.01);
            }
        }).start();
    }

    public void reset() {
        resetX = x;
    }

    public double getX() {
        return Normalize_Gryo_Value(x - resetX);
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public boolean getPossibleError() {
        return possibleError;
    }

    @Override
    public double getAsDouble() {
        return getX();
    }

}
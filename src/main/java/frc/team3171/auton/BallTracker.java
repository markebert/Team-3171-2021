package frc.team3171.auton;

//FRC Import
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

//OpenCV Imports
import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * @author Evan Lockwood
 */
public class BallTracker{

    /*private UsbCamera feederCamera;
    private CvSink cvSink;
    private CvSource outputStream;
    private Mat source;
    private Mat output;*/

    Thread m_visionThread;


    public BallTracker(){

    }

    public void galacticSearch(){
        /*m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              //UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
              // Set the resolution
              //camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              //CvSink cvSink = CameraServer.getInstance().getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              //CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              //Mat mat = new Mat();
              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
            m_visionThread.setDaemon(true);
            m_visionThread.start();*/
            new Thread(() -> {
                UsbCamera feederCamera = CameraServer.getInstance().startAutomaticCapture();
                feederCamera.setResolution(640, 480);
                feederCamera.setFPS(30);  
                CvSink cvSink = CameraServer.getInstance().getVideo();
                CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
                Mat source = new Mat();
                Mat output = new Mat();
                while(!Thread.interrupted()) {
                  if (cvSink.grabFrame(source) == 0) {
                    continue;
                  }
                  Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                  outputStream.putFrame(output);
                }
              }).start();
    }

    /*public void colorDetector(){

    }*/

}


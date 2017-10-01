package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.Log;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/**
 * Created by kawaiiPlat on 10/1/2017.
 */

public class OpenCV {

    public static void cvManagerInit(Context context) {
        BaseLoaderCallback callback = new BaseLoaderCallback(context) {
            @Override public void onManagerConnected(int status) {
                switch(status) {
                    case LoaderCallbackInterface.SUCCESS:
                        Log.i(FtcRobotControllerActivity.TAG, "OpenCV Manager Connected");
                        break;
                    case LoaderCallbackInterface.INIT_FAILED:
                        Log.i(FtcRobotControllerActivity.TAG, "Init Failed");
                        break;
                    case LoaderCallbackInterface.INSTALL_CANCELED:
                        Log.i(FtcRobotControllerActivity.TAG, "Install Canceled");
                        break;
                    case LoaderCallbackInterface.INCOMPATIBLE_MANAGER_VERSION:
                        Log.i(FtcRobotControllerActivity.TAG, "Incompatible Manager Version");
                        break;
                    case LoaderCallbackInterface.MARKET_ERROR:
                        Log.i(FtcRobotControllerActivity.TAG, "Market Error");
                        break;
                    default:
                        Log.i(FtcRobotControllerActivity.TAG, "OpenCV Manager Install");
                        super.onManagerConnected(status);
                        break;
                }
            }
        };

        if(!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, context, callback);
        } else {
            callback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public static Mat performHSVThreshold(Mat input, Mat output, double hueMin, double hueMax, double satMin, double satMax, double valMin, double valMax) {
        Imgproc.cvtColor(input, output, Imgproc.COLOR_BGR2HSV);
        Core.inRange(input, new Scalar(hueMin, satMin, valMin), new Scalar(hueMax, satMax, valMax), output);
        return output;
    }

    public static List<MatOfPoint> performFindContours(Mat input, List<MatOfPoint> output, boolean externalOnly) {
        Mat hierarchy = new Mat();
        output.clear();
        int mode = externalOnly ? Imgproc.RETR_EXTERNAL : Imgproc.RETR_LIST;
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, output, hierarchy, mode, method);
        return output;
    }

    public static List<MatOfPoint> performConvexHulls(List<MatOfPoint> input, List<MatOfPoint> output) {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        for(int i = 0; i < input.size(); i++) {
            final MatOfPoint contour = input.get(i);
            final MatOfPoint mopHull = new MatOfPoint();
            Imgproc.convexHull(contour, hull);
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for(int j = 0; j < hull.size().height; j++) {
                int index = (int) hull.get(j, 0)[0];
                double[] point = new double[] {contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            output.add(mopHull);
        }
        return output;
    }

    public static List<MatOfPoint> performFilterContours(List<MatOfPoint> input, List<MatOfPoint> output,
                                                         double minArea, double minPerimeter, double minWidth, double maxWidth,
                                                         double minHeight, double maxHeight, double minSolidity, double maxSolidity,
                                                         double minVerticies, double maxVerticies, double minRatio, double maxRatio) {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        for (int i = 0; i < input.size(); i++) {
            final MatOfPoint contour = input.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth) continue;
            if (bb.height < minHeight || bb.height > maxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int)hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < minSolidity || solid > maxSolidity) continue;
            if (contour.rows() < minVerticies || contour.rows() > maxVerticies)	continue;
            final double ratio = bb.width / (double)bb.height;
            if (ratio < minRatio || ratio > maxRatio) continue;
            output.add(contour);
        }

        return output;
    }

    public static Mat matFromVuforia(VuforiaLocalizer vuforia) {
        try {
            Log.d("cv3", "Getting frame");
            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
            Log.d("cv3", "Gotten Frame");
            long numImages = frame.getNumImages();
            Log.d("cv3", "num img");
            Image rgb = null;

            for(int i = 0; i < numImages; i++) {
                Log.d("cv3", "Format: " + frame.getImage(i).getFormat());
                if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);
                    break;
                }
            }

            if(rgb == null) {
                Log.d("cv3", "RGB was null");
                return null;
            }

            Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(rgb.getPixels());

            Mat mat = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
            Utils.bitmapToMat(bm, mat);

            Log.d("cv3", "Done closing and exiting");

            frame.close();

            Log.d("cv3", "Closed");

            return mat;
        } catch(Exception ex) {
            Log.d("cv3", "Ex", ex);
            return null;
        }
    }
}

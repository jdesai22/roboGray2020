package org.firstinspires.ftc.teamcode;

import java.io.File;
import java.io.IOException;
//import java.awt.image.BufferedImage;
//import javax.imageio.ImageIO;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.CvType;
import org.opencv.imgcodecs.Imgcodecs;

public class ImageRecognition {
    public static void main(String[] args) {
        try {
            System.out.println("Image Loaded");

//            System.loadLibrary( Core.NATIVE_LIBRARY_NAME );
//
//            //Instantiating the Imagecodecs class
//            Imgcodecs imageCodecs = new Imgcodecs();
//
//            //Reading the Image from the file
//            Mat matrix = imageCodecs.imread("src/main/java/org/firstinspires/ftc/teamcode/ImageTests/imgs/iFF4rnX.jpg");
//
//
//            System.out.println(matrix);
            System.out.println("Image Loaded");
        } catch (Exception e) {
            System.out.print(e);
        }

    }

    private int position() {
        return 0;
    }
}

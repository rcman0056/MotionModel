import javax.imageio.ImageIO;
import javax.imageio.ImageReader;
import javax.imageio.stream.FileImageInputStream;
import javax.imageio.stream.ImageInputStream;
import javax.media.jai.JAI;
import javax.media.jai.RenderedOp;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Iterator;


/**
 * Created by daniel on 08/09/16.
 */
public class readImage {
    // method to read in an image as a pixel array
    public static int[] getImage(String path) throws IOException {

        BufferedImage img = null;
        try {
            img = ImageIO.read(new File(path));
        } catch (IOException e) {
            e.printStackTrace();
        }
        File f = new File(path);
        if (f.exists()) {

            int width = img.getWidth();
            int height = img.getHeight();
            int[][][] result = new int[height][width][3];
            int[] imageBytes = new int[height * width + 2];
            imageBytes[0] = height; //is int, not byte
            imageBytes[1] = width;
            for (int row = 0; row < height; row++) {
                for (int col = 0; col < width; col++) {
                    //grab binary getData for each color stream for a pixel
                    int color = img.getRGB(col, row);
                    //separate into integer values
                    int blue = color & 0xff;
                    int green = (color & 0xff00) >> 8;
                    int red = (color & 0xff0000) >> 16;
                    //add to 3D array
                    result[row][col][0] = red;
                    result[row][col][1] = blue;
                    result[row][col][2] = green;
                    //add to 1D intensity array
                    int avgIntensity = (int) Math.rint((red + blue + green) / 3);
                    imageBytes[2 + (row) + (height * col)] = avgIntensity;
                }
            }
            return (imageBytes);
        } else {
            int temp[] = {0};
            return (temp);
        }
    }
}

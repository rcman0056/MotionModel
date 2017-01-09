/**
 * Created by daniel on 07/09/16.
 */

import golem.matrix.Matrix;

import java.nio.ByteBuffer;

public class runVO {
    static {
        //System.setProperty("java.library.path", "/home/daniel/workspace/optical_flow/Carson-OpticalFlow/src");
        //System.load(System.getProperty("user.dir")+"/Carson-OpticalFlow/librunOpticalFlow.so");
        System.load("/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/src/main/modules/VOAll/librunVO.so");
    }


    // Declare native methods
    private native double[] VO(ByteBuffer buffer);

    //process an image
    public static double[] VO(int[] firstImage, int[] secondImage, double h, double[] f, double[] c,
                                       Matrix<Double> firstCamToNav, Matrix<Double> secondCamToNav, boolean[] params) {

        int nx = firstImage[0];
        int ny = firstImage[1];

        boolean useFeatures = params[0];
        boolean inertialAiding = params[1];

        int numBools = 2;
        int numInts = 2+numBools;
        int intBytes = 2;
        int numDoubles = 5;
        int doubleBytes = 8;
        int numDCMs = 2;
        int DCMBytes = 72;
        int numImages = 2;
        int imageBytes = nx * ny;


        int numBytes = numImages * imageBytes + numInts * intBytes + numDoubles * doubleBytes + numDCMs * DCMBytes;
        byte[] bytes = new byte[numBytes];

        //encode the first 50 bytes with the height above ground and focal lengths and optical centers
        int index = 0;
        bytes = byteConversion.addDoubleToByteArray(h, bytes, index);
        index += doubleBytes;
        bytes = byteConversion.addDoubleToByteArray(f[0], bytes, index);
        index += doubleBytes;
        bytes = byteConversion.addDoubleToByteArray(f[1], bytes, index);
        index += doubleBytes;
        bytes = byteConversion.addDoubleToByteArray(c[0], bytes, index);
        index += doubleBytes;
        bytes = byteConversion.addDoubleToByteArray(c[1], bytes, index);
        index += doubleBytes;

        //encode the next 72 bytes with the first DCM
        bytes = byteConversion.addDCMToByteArray(firstCamToNav, bytes, index);
        index += DCMBytes;

        //encode the next 72 bytes with the second DCM
        bytes = byteConversion.addDCMToByteArray(secondCamToNav, bytes, index);
        index += DCMBytes;

        //encode the next 4 bytes with the horizontal and vertical resolution
        bytes = byteConversion.addIntToByteArray(nx, bytes, index);
        index += intBytes;
        bytes = byteConversion.addIntToByteArray(ny, bytes, index);
        index += intBytes;

        //fill up half of the remaining space with the first image
        for (int i = 0; i < imageBytes; i++) {
            bytes[i + index] = (byte) firstImage[i + 2];
        }
        index += imageBytes;

        //fill up the remaining space with the second image
        for (int i = 0; i < imageBytes; i++) {
            bytes[i + index] = (byte) secondImage[i + 2];
        }
        index += imageBytes;

        //add the boolean parameters
        int bool1 = 0;
        int bool2 = 0;
        if (useFeatures) {
            bool1 = 1;
        }
        if (inertialAiding){
            bool2 = 1;
        }
        bytes = byteConversion.addIntToByteArray(bool1, bytes, index);
        index += intBytes;
        bytes = byteConversion.addIntToByteArray(bool2, bytes, index);
        index += intBytes;


        ByteBuffer buffer = ByteBuffer.allocateDirect(numBytes);
        buffer.put(bytes);

        buffer.flip();
        double[] deltaPos = new runVO().VO(buffer);
        return deltaPos;
    }
}


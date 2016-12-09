import golem.matrix.Matrix;

import java.nio.ByteBuffer;

/**
 * Created by daniel on 23/11/16.
 */
public class byteConversion {

    public static byte[] addIntToByteArray(int integerIn, byte[] byteArray, int index) {
        /*
        byte[] bytes = new byte[2];
        ByteBuffer.wrap(bytes).putInt(integerIn);
        for (int i=0; i<2; i++) {
            byteArray[i+index] = bytes[i];
        }
*/

        byteArray[index+0] = (byte) (integerIn >> 8);
        byteArray[index+1] = (byte) integerIn;

        return byteArray;
    }

    public static byte[] addDoubleToByteArray(double doubleIn, byte[] byteArray, int index) {
        byte[] bytes = new byte[8];
        ByteBuffer.wrap(bytes).putDouble(doubleIn);
        for (int i=0; i<8; i++) {
            byteArray[i+index] = bytes[i];
        }
        return byteArray;
    }

    public static byte[] addDCMToByteArray(Matrix<Double> DCMIn, byte[] byteArray, int index) {
        byte[] bytes = new byte[72];
        for(int i=0;i<3;i++) {
            for(int j=0;j<3;j++) {
                ByteBuffer.wrap(bytes).putDouble(DCMIn.get(i,j));
                for (int k=0; k<8; k++) {
                    byteArray[k+index] = bytes[k];
                }
                index += 8;
            }
        }
        for (int i=0; i<72; i++) {
            byteArray[i+index] = bytes[i];
        }
        return byteArray;
    }

}

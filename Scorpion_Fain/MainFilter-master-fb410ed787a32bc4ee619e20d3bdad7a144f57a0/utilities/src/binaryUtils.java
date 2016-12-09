import ch.qos.logback.core.encoder.ByteArrayUtil;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class binaryUtils {
    public static void read(String filename) throws IOException {
        Path path = Paths.get(filename);
        byte[] bytes = Files.readAllBytes(path);
        int stop = bytes.length/8;
        double data[] = new double[stop];
        for (int i = 0;i<stop;i++) {
            byte tempBytes[] = new byte[8];
            for (int j = 0;j<8;j++) {
                tempBytes[j] = bytes[i+j];
            }
            data[i] = ByteBuffer.wrap(tempBytes).getDouble();
        }

    }
}

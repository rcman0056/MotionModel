import golem.*
import golem.containers.Time
import golem.matrix.Matrix
import modules.aux_data.PosVelAuxData
import navutils.rpyToDcm
import preprocessors.Preprocessor
import scorpion.filters.sensor.containers.Measurement

/**
 * This preprocessor algorithm calculates a velocity update from a series of images.
 *
 * An array of camera calibration parameters is necessary to initialize the preprocessor. The array should contain the
 * following values:
 *
 * camCal = doubleArrayOf(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy)
 *
 * Once the preprocessor is initialized, each image can be added individually. The preprocessor will calculate the first
 * pose after the third image is added and the first velocity after the fourth image, but reliable velocity measurements
 * can be obtained after the fifth image is added. The following inputs are necessary to advance the visual odometry:
 *
 * timeStep: the elapsed time between the previous image and the image being added, necessary to calculate the velocity
 *
 * image: a 1D array of integers where the first int is the height (h pixels) of the image, the second is the width
 * (w pixels), and each of the following h*w ints is a grayscale pixel value of the image. Thus the first row of pixels
 * exists at indeces in the range 2..1+w, the second row is indexed by the range 2+w..1+2*w, and so on.
 *
 * scale: the distance from the camera to the ground. Can be adjusted to account for biases in the SVO
 *
 * camToNav: a DCM which rotates the measurements from the camera from to the navigation frame
 *
 * t: a Scorpion Time object
 *
 * filterVel: the filter's estimated velocity
 *
 * processor: a string identifying the label of the measurement processor that will accept the preprocessor's velocity
 * measurement
 *
 * measCov: a 3x3 matrix containing the covariance values of the measurements
 *
 *
 * addFrame outputs a measurement object that can be fed into a measurement processor
 *
 * getDebugInfo returns debug_data, a 1x9 matrix rotated velocity, svo attitude, and svo position
 *
 */

//class preprocessorSVO constructor(args: Array<String>){
class OpticalFlow_Carson : Preprocessor<String> {
    override fun process(rawMeas: String): Measurement {
        throw UnsupportedOperationException()
    }

    var debug_data = zeros(1,3) //record rotated velocity

    fun runOpticalFlow(timeStep: Double, firstImage: IntArray, secondImage: IntArray, h: Double, f: DoubleArray,
                       c: DoubleArray, firstCamToNav: Matrix<Double>, secondCamToNav: Matrix<Double>, t: Time,
                       filterVel: Matrix<Double>, processor: String, measCov: Matrix<Double>): Measurement {


        //get translation vector
        val translation = runOpticalFlow.opticalFlow(firstImage, secondImage, h, f, c, firstCamToNav, secondCamToNav)

        //print results to console
        if (false) {
            print(translation[0])
            print("\t")
            print(translation[1])
            print("\t")
            println(translation[2])
        }

        //calculate velocity
        var vel = zeros(3, 1)
        vel[0] = translation[0] / timeStep
        vel[1] = translation[1] / timeStep
        vel[2] = translation[2] / timeStep

        debug_data = vel

        val velErr = vel - filterVel
        val aux = PosVelAuxData(PosVelAuxData.PosVelMeasurementTypes.VELOCITY)
        val opticalFlowMeasurement = Measurement(processor, t, t, velErr, aux, measCov)
        return opticalFlowMeasurement
    }

    fun getDebugData(): Matrix<Double> {
        return debug_data
    }
}
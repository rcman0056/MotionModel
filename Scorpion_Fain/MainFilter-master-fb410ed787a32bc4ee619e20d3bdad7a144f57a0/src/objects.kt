import golem.mat
import golem.zeros

/**
 * Created by daniel on 07/12/16.
 */
/**
 * Object that contains truth data.
 */
object initalTruthValues {
    // Initial position, velocity and attitude
    var truthPos0 = mat[LAT0, LON0, ALT0]  // Lat (rad), Lon (rad), Alt (m)
    var truthVel0 = zeros(1, 3)
    var truthRot0 = zeros(3)
}

/**
 * Object that data for SVO measurements
 */
object avgSVOMeasurementBuffer {
    // Initial position, velocity and attitude
    var collected = zeros(NUM_MEAS_AVG, 3)
    var avg = zeros(1, 3)
    var count = 0
}

object cameraCalibrationParams {
    var width = 0.0
    var height = 0.0
    var fx = 0.0
    var fy = 0.0
    var cx = 0.0
    var cy = 0.0
}
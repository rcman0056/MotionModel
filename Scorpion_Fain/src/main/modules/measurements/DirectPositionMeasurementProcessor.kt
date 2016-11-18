package modules.measurements

import golem.*
import golem.matrix.*
import navutils.containers.NavSolution
import navutils.deltaLatToNorth
import navutils.deltaLonToEast
import scorpion.filters.sensor.containers.Measurement
import scorpion.filters.sensor.containers.MeasurementModel

/**
 * This class provides the means of updating Pinson15NED error states with a
 * measurement vector in which the measurements are 3x1 direct latitude,
 * longitude and altitude measurements (in rad, rad, m respectively) and the measurement
 * uncertainty matrix is in m^2 NED. Accepts AuxData in the form of the NavSolution class
 * which is used to form differenced measurements. The NavSolution.pose.pos must contain
 * position values in latitude (rad), longitude (rad) and altitude (m).
 */
class DirectPositionMeasurementProcessor : PositionMeasurementProcessor {

    private var referenceSolution: NavSolution? = null  // Storage for asynchronous aux data

    constructor(label: String, pinsonLabel: String) : super(label, pinsonLabel)

    /**
     * Generates the model that relates a 3x1 position measurement of latitude,
     * longitude and altitude measurements (in rad, rad, m respectively) to a
     * Pinson15NED state block..
     *
     * @param meas: Measurement that contains 3x1 position measurement.
     * @param xhat: State block estimates for block described by pinsonLabel.
     * @param P: State block covariance for block described by pinsonLabel.
     */
    override fun generateModel(meas: Measurement, xhat: Matrix<Double>, P: Matrix<Double>): MeasurementModel {
        // Need an immutable copy of this value for null safety checking
        val currentRefSolution = this.referenceSolution

        // The auxData attached to the measurement gets preference; use any available auxData
        // stored locally if none with measurement. Cannot continue if both of these are null.
        if (meas.auxData != null) {
            meas.measurementData = diffWgsGpsWithWgsNavSolution(meas.measurementData, meas.auxData as NavSolution)

        } else if (currentRefSolution != null) {
            // Check if processor was sent aux data asynchronously and use if available.
            // TODO: In deployment, we may want to check the times and make sure they match within some threshold
            meas.measurementData = diffWgsGpsWithWgsNavSolution(meas.measurementData, currentRefSolution)
        } else {
            error("Must provide DirectPositionMeasurementProcessor auxData" +
                  " in the form of a current NavSolution before a" +
                  " measurementModel can be generated.")
        }
        return super.generateModel(meas, xhat, P)
    }

    /**
     * Store a NavSolution that is the position calculated by the inertial. This
     * may be passed in here asynchronously to be used with the next measurement
     * or directly on the measurements auxData field during a call to [generateModel].
     *
     * @param n: A NavSolution that contains a 3x1 position measurement of
     * latitude, longitude and altitude measurements (in rad, rad, m
     * respectively) in the pose.pos field.
     */
    fun receiveAuxData(n: NavSolution) {
        this.referenceSolution = n
    }

    /**
     * Differences WGS position values and converts to NED.
     *
     * @param m: 3x1 matrix of latitude, longitude and altitude measurements
     * (in rad, rad, m respectively), such as from GPS.
     * @param n: A NavSolution that contains a 3x1 position measurement of
     * latitude, longitude and altitude measurements (in rad, rad, m
     * respectively) in the pose.pos field.
     *
     * @return: m - n.pose.pos, converted to NED coordinates (m).
     */
    private fun diffWgsGpsWithWgsNavSolution(m: Matrix<Double>, n: NavSolution): Matrix<Double> {
        var pos = n.pose.pos
        var dNorth = deltaLatToNorth(m[0] - pos[0], pos[0])
        var dEast = deltaLonToEast(m[1] - pos[1], pos[0])
        // Error in the down direction is the opposite sign of the altitude difference
        var dDown = pos[2] - m[2]
        return mat[dNorth, dEast, dDown].T
    }
}
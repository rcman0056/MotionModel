package modules.measurements

import golem.*
import golem.matrix.*
import navutils.containers.Vector3
import scorpion.filters.sensor.containers.Measurement
import scorpion.filters.sensor.containers.MeasurementModel
import scorpion.filters.sensor.containers.MeasurementProcessor
import scorpion.util.getNumericalJacobian

/**
 * A measurement processor that accepts biased range updates of a direct-position.
 * Assumes you have a 1x1 bias StateBlock and 3x1 position StateBlock in your filter. It then calculates:
 *
 * z = norm(position-beacon)+bias
 *
 * @param label The label uniquely identifying this particular processor.
 * @param biasLabel The label uniquely identifying the bias StateBlock.
 * @param posLabel The label uniquely identifying the direct 3-position StateBlock.
 * @param beaconLocation The location we are getting ranges to.
 */
class BiasedRangeProcessor : MeasurementProcessor {
    override var label: String
    override var stateBlockLabels: Array<String>

    var beaconLocation: Vector3

    constructor (label: String,
                 biasLabel: String,
                 posLabel: String,
                 beaconLocation: Vector3)  {

        this.beaconLocation = beaconLocation
        this.label = label
        this.stateBlockLabels = arrayOf(posLabel, biasLabel)
    }

    override fun generateModel(meas: Measurement, xhat: Matrix<Double>, P: Matrix<Double>): MeasurementModel {

        // Create the non-linear measurement function
        fun h(x: Matrix<Double>): Matrix<Double> {
            val biasedRange = sqrt(((x[0] - beaconLocation.x) pow 2) + ((x[1] - beaconLocation.y) pow 2) + ((x[2] - beaconLocation.z) pow 2)) + x[3]

            // Creates a 1x1 matrix from double
            return mat[biasedRange]
        }

        val z = meas.measurementData
        val h = ::h
        val H = getNumericalJacobian(::h)
        val R = meas.measurementCov

        return MeasurementModel(z, h, H, R)
    }

    /**
     * Unused for this processor.
     */
    override fun receiveAuxData(auxData: Any) {
    }
}



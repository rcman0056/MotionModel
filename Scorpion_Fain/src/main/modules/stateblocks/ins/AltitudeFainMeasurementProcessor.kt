package main.modules.stateblocks.ins

import golem.*
import golem.matrix.Matrix
import modules.stateblocks.ins.MotionModelAuxData
import scorpion.filters.sensor.containers.Measurement
import scorpion.filters.sensor.containers.MeasurementModel
import scorpion.filters.sensor.containers.MeasurementProcessor


/**
 *  A measurement processor that accepts a direct measurement on a 1x1 stateblock.
 *
 */
class AltitudeFainMeasurementProcessor : MeasurementProcessor {
    override var label: String
    override var stateBlockLabels: Array<String>
    private var auxData: MotionModelAuxData? = null

    /**
     * This block receives Motion Model inputs data via calls to this function. The parameter must be of type
     * [MotionModelAuxData], which is a container passing in motion model inputs
     */
    override fun receiveAuxData(auxData: Any) {
        if (auxData is MotionModelAuxData) {
            this.auxData = auxData
        } else {
            throw UnsupportedOperationException("This class only accepts MotionModelAux instances")
        }
    }
    /**
     *  Constructor takes an identifying label for this processor.
     *  @param label The label uniquely identifying this particular processor.
     *  @param blockLabel the label of the StateBlock we are updating.
     *  @param HValue the value H in the measurement equation z = Hx
     */
    constructor (label: String,
            //HValue: Double,
                 blockLabel: String) {
        this.label = label
        this.stateBlockLabels = arrayOf(blockLabel)
        //this.HValue = HValue
        this.blockLabel = blockLabel
    }

    //val HValue: Double
    val blockLabel: String

    // Generates z, h, H, and R for the filter to use during updates.
    override fun generateModel(meas: Measurement, xhat: Matrix<Double>, P: Matrix<Double>): MeasurementModel {




        // Pass raw data into filter as measurement
        //Correct for values beyond 0-2PI
        var z = meas.measurementData
        // Constructs a 1x1 matrix for H using the parameter HValue we received before
        var H = mat[0,0,0,0,0,0,0,1,0]
        // Use the measurement container's covariance
        var R = meas.measurementCov
        // Make a inner function "h" to return to the filter
        fun h(x: Matrix<Double>): Matrix<Double> {
            return H * x
        }

        // ::h is a reference to the function "h"
        return MeasurementModel(z, ::h, H, R)
    }

    // Unused for this example, so throw an error if someone tries to use it

}
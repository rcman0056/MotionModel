package main.modules.stateblocks.ins

import golem.*
import golem.matrix.Matrix
import modules.stateblocks.ins.FainMeasurements
import navutils.deltaLatToNorth
import navutils.deltaLonToEast
import scorpion.filters.sensor.containers.Measurement
import scorpion.filters.sensor.containers.MeasurementModel
import scorpion.filters.sensor.containers.MeasurementProcessor


/**
 *
 *
 */
class VOMeasurementProcessor : MeasurementProcessor {
    override var label: String
    override var stateBlockLabels: Array<String>
    private var auxData: FainMeasurements? = null

    /**
     * This block receives Motion Model inputs data via calls to this function. The parameter must be of type
     * [MotionModelAuxData], which is a container passing in motion model inputs
     */
    override fun receiveAuxData(auxData: Any) {
        if (auxData is FainMeasurements) {
            this.auxData = auxData
        } else {
            throw UnsupportedOperationException("This class only accepts FainMeasurements instances")
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
        this.blockLabel = blockLabel
    }

    //val HValue: Double
    val blockLabel: String

    // Generates z, h, H, and R for the filter to use during updates.
    override fun generateModel(meas: Measurement, xhat: Matrix<Double>, P: Matrix<Double>): MeasurementModel {

        var data = meas.measurementData
        var dt = data[0]

        var Vx = data[1]
        var Vy = data[2]
        var Vz = data[3] //Not used but could be used as an Alt update....Vz/dt


        var Ground_Speed = pow((Vx * Vx) + (Vy * Vy), .5) / dt
        var Course_ang = Math.atan2(Vy,Vx)
        var z = mat[Ground_Speed end Course_ang]
        // Constructs a 1x1 matrix for H using the parameter HValue we received before
        var H = mat[0, 0, 1, 0, 0, 0, 0, 0, 0 end
                    0, 0, 0, 1, 0, 0, 0, 0, 0]
        // Use the measurement container's covariance
        var R = meas.measurementCov  //sigma^2

        // Make a inner function "h" to return to the filter
        fun h(x: Matrix<Double>): Matrix<Double> {
            return H * x
        }

        // ::h is a reference to the function "h"
        return MeasurementModel(z, ::h, H, R)
    }

    // Unused for this example, so throw an error if someone tries to use it

}

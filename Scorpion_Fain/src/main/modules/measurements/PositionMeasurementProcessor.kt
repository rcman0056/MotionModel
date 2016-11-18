package modules.measurements

import golem.*
import golem.matrix.*
import scorpion.filters.sensor.containers.Measurement
import scorpion.filters.sensor.containers.MeasurementModel
import scorpion.filters.sensor.containers.MeasurementProcessor

/**
 * A measurement processor that accepts position updates for a pinson15 block.
 * The measurements passed in must be differenced and in the same units as the
 * Pinson block position error states.
 *
 * @param label The label uniquely identifying this particular processor.
 * @param pinsonLabel The label of the pinson StateBlock we are updating.
 */
open class PositionMeasurementProcessor : MeasurementProcessor {
    override var label: String
    override var stateBlockLabels: Array<String>

    constructor(label: String,
                pinsonLabel: String) {
        this.label=label
        this.stateBlockLabels = arrayOf(pinsonLabel)
    }

    /**
     * Generates a model that relates a 3x1 position measurement to a Pinson state block.
     */
    override fun generateModel(meas: Measurement, xhat: Matrix<Double>, P: Matrix<Double>): MeasurementModel {
        checkMeasurementSize(meas)
        var H = fillHWithEyeBlock(3, xhat.numRows())
        return linearModel(meas, H)
    }

    /**
     * Ensure that an input measurement is a column vector with the expected
     * number of rows.
     *
     * @param meas: Measurement to check.
     * @param expected: Number of rows meas.measurementData should have.
     * @throws IllegalArgumentException: If size of meas.measurementData
     * doesn't match expected.
     */
    fun checkMeasurementSize(meas: Measurement, expected: Int = 3){
        if (meas.measurementData.numRows() != expected)
            throw IllegalArgumentException("Must be a ${expected}x1 measurement")
    }

    /**
     * Generates a H matrix populated with an identity block inserted at
     * a location denoted by an offset parameter.
     *
     * @param measSize: Size of measurement H is with respect to.
     * @param offset: Number of columns within H to place the first element
     * of the identity matrix.
     * @param stateSize: Number of states in the state vector that H is with respect to.
     * @return: A measSize-by-stateSize Matrix H such that z_pred = H*x.
     * @throws Exception: If the offset passed in would cause the identity
     * matrix to surpass the bounds of H when placed.
     */
    fun fillHWithEyeBlock(measSize: Int = 3, stateSize: Int = 15, offset: Int = 0): Matrix<Double>{
        if(offset > stateSize - measSize)
            throw Exception("Offset must be no greater than ${stateSize - measSize} for measurement of size $measSize")
        return fill(measSize, stateSize) { row, col -> if (row + offset == col) 1.0 else 0.0 }
    }

    /**
     * Packages up a measurement model when h(x) = H*x.
     *
     * @param meas: Input measurement.
     * @param H: Linearized measurement model matrix.
     * @return: Measurement model where h = H*x.
     */
    fun linearModel(meas: Measurement, H: Matrix<Double>): MeasurementModel{
        return MeasurementModel(meas.measurementData,
                { H * it }, H, meas.measurementCov)
    }

    /**
     * Unused for this processor.
     */
    override fun receiveAuxData(auxData: Any) {
        throw UnsupportedOperationException()
    }

}

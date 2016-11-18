package modules.measurements

import golem.matrix.Matrix
import modules.aux_data.PosVelAuxData
import scorpion.filters.sensor.containers.Measurement
import scorpion.filters.sensor.containers.MeasurementModel
import modules.aux_data.PosVelAuxData.PosVelMeasurementTypes.*

class PosVelMeasurementProcessor : PositionMeasurementProcessor {

    constructor(label: String, pinsonLabel: String) : super(label, pinsonLabel)

    /**
     * Generates the model that relates a 3x1 position, 3x1 velocity, or 6x1 position
     * and velocity measurement to a Pinson15 state block, depending on the contents
     * of the auxData attached to the measurement.
     *
     * @param meas: Measurement that contains 3x1 position, 3x1 velocity,
     * or 6x1 position and velocity values, and an instance of PosVelAuxData
     * describing the contents of the measurement.
     * @param xhat: State block estimates for block described by pinsonLabel.
     * @param P: State block covariance for block described by pinsonLabel.
     */
    override fun generateModel(meas: Measurement, xhat: Matrix<Double>, P: Matrix<Double>):
            MeasurementModel {
        val m = meas
        val aux = m.auxData
        when(aux){
            is PosVelAuxData ->{
                if(aux.contents == POSITION){
                    return super.generateModel(meas, xhat, P)
                }else if(aux.contents == VELOCITY){
                    checkMeasurementSize(meas)
                    return linearModel(meas, fillHWithEyeBlock(3, xhat.numRows(), 3))
                }else if(aux.contents == BOTH){
                    checkMeasurementSize(meas, 6)
                    return linearModel(meas, fillHWithEyeBlock(6, xhat.numRows()))
                }
            }
        }
        throw Exception("Must provide PosVelAuxData with measurements sent to" +
                " PosVelMeasurementProcessor.")
    }

}






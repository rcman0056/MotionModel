/**
 * Created by ucav on 8/15/2016.
 */
package modules.measurements

import golem.*
import golem.matrix.*
import navutils.containers.NavSolution
import navutils.deltaLatToNorth
import navutils.deltaLonToEast
import scorpion.filters.sensor.containers.Measurement
import scorpion.filters.sensor.containers.MeasurementModel
import scorpion.filters.sensor.containers.MeasurementProcessor
import java.util.*

open class StereoAttitudeMeasurementProcessor: MeasurementProcessor {
    override var label: String
    override var stateBlockLabels: Array<String>

    constructor(label: String,
                pinsonLabel: Array<String>){
        this.label=label
        this.stateBlockLabels=pinsonLabel
    }


    override fun generateModel(meas: Measurement, xhat: Matrix<Double>, P: Matrix<Double>): MeasurementModel {
        //preloading H matrix
        var H = zeros(3,30)

        //reading in the aux data.  If it is not an Array<NavSolution> it will error
        val a = meas.auxData as? Array<NavSolution>?: error("Type must be Array<NavSolution>")
        //var nL=a[0]
        //var nR=a[1]
        var insL=a[0]
        var insR=a[1]

        //putting measurement in correct body frame
        var NLU=meas.measurementData
        //Virtual world body is nose, left wing, up vs nose, right wing, down
        var Cbv=mat[1,0,0 end 0,-1,0 end 0,0,-1]
        var NRD=Cbv*NLU

        var measurement=determineMeasurement(NRD,insL,insR)

        H[0..2,6..8]=-eye(3)
        H[0..2,21..23]=eye(3)

        var cov=meas.measurementCov
        return MeasurementModel(measurement, // pass-through
                {H * it},
                H,
                cov)
    }
    //First call to receive AuxData must be auxData:Any
    override fun receiveAuxData(auxData: Any) {
        throw UnsupportedOperationException()
    }

    fun determineMeasurement(m:Matrix<Double>,insL:NavSolution,insR:NavSolution): Matrix<Double> {

        // Need both current solutions
        var posL=insL.pose.pos
        var posR=insR.pose.pos

        var Cerr=m*insL.pose.rotMat*insR.pose.rotMat.T
        var measurements=mat[Cerr[3,2],Cerr[1,3],Cerr[2,1]].T

        return measurements
    }
}

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
class SimulatedRangeMeasurementProcessorTwo : MeasurementProcessor {
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
        var Curr_Meas = meas.auxData as FainMeasurements

        //Calculate an XYZ in NEU frame where the origin is the same as the filters being the first GPS measurement received.
        var CenterX = Curr_Meas.simulated_range_CenterNEU_Two[0]
        var CenterY = Curr_Meas.simulated_range_CenterNEU_Two[1]
        var CenterZ = Curr_Meas.simulated_range_CenterNEU_Two[2]
        var Speed = Curr_Meas.simulated_range_ground_speed_Two
        var Radius = Curr_Meas.simulated_range_radius_Two
        var dt_total = Curr_Meas.simulated_range_total_dt_Two
        var pi = Math.PI

        var Angle_diff=-(dt_total*Speed)/(2*pi*Radius)+Curr_Meas.Circumference_Offset_rads    //+pi/2; //add the pi/2 to get a plane offset //Negative ang makes the plane fly clock wise
        var X_sim=cos(Angle_diff)*Radius+CenterX
        var Y_sim=sin(Angle_diff)*Radius+CenterY
        var Z_sim=CenterZ                   //Alt stays the same


        //Calculated the current GPS position into an NEU Frame
        var current_gps = mat[Curr_Meas.GPS_Linux_time, Curr_Meas.GPS_lat, Curr_Meas.GPS_lon, Curr_Meas.GPS_height_agl]
        //Convert to NEU using first received GPS value as origin

        var GPS_Y_NEU =      deltaLatToNorth((current_gps[1] - Curr_Meas.GPS_origin_lat) * Math.PI / 180, current_gps[1] * Math.PI / 180, current_gps[3]) //Diff in rads,Current lat,Estimated ALT
        var GPS_X_NEU =      deltaLonToEast((current_gps[2] - Curr_Meas.GPS_origin_lon) * Math.PI / 180, current_gps[1] * Math.PI / 180, current_gps[3])
        var GPS_Z_NEU =      current_gps[3]

        //Calculate truth range
        var North_delta = Y_sim - GPS_Y_NEU
        var East_delta  = X_sim - GPS_X_NEU
        var Alt_delta   = Z_sim - GPS_Z_NEU
        var z =  mat[pow(pow(North_delta,2)+pow(East_delta,2)+pow(Alt_delta,2),.5)]


        //Calculate Non-Linear H
        var North_delta1 = xhat[0] - Y_sim
        var East_delta1  = xhat[1] - X_sim
        var Alt_delta1   = xhat[7] - Z_sim
        // h = sqrt((North Diff^2)+(East Diff^2)+(Alt Diff^2))
        var h1 = pow(pow(North_delta1, 2) + pow(East_delta1, 2) + pow(Alt_delta1, 2), .5)

        println("z=" + z.toString() + "/t" + "h1=" + h1.toString())

        var H = mat[North_delta1 / h1, East_delta1 / h1, 0, 0, 0, 0, 0, Alt_delta1 / h1, 0] //linearized measurement
        // Use the measurement container's covariance
        var R = meas.measurementCov
        //println("z=" + z + "\n" + "xhat=" + xhat[6] * (180 / Math.PI) + "\n" + "meas=" + meas.measurementData)
        // Make a inner function "h" to return to the filter
        fun h(x: Matrix<Double>): Matrix<Double> {
            var North_delta = x[0] - Y_sim
            var East_delta  = x[1] - X_sim
            var Alt_delta =   x[7] - Z_sim
            // h = sqrt((North Diff^2)+(East Diff^2)+(Alt Diff^2))


            var h = pow(pow(North_delta, 2) + pow(East_delta, 2) + pow(Alt_delta, 2), .5)

            return mat[h]
        }
        return MeasurementModel(z, ::h, H, R)
    }
}

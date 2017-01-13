package modules.stateblocks.ins

import golem.matrix.*
//import navutils.containers.NavSolution

/**
 * Passes into a [Pinson15Block] the set of current values from a running INS mechanization to linearize about.
 *
 * @param force NED specific force vector
 * @param navSolution the current solution from the INS mechanization
 */
data class FainMeasurements(  var range: Double,
                              var range_time: Double,
                              var range_Lat: Double,
                              var range_Lon: Double,
                              var range_alt: Double,
                              var simulated_range: Double,
                              var simulated_range_total_dt: Double,
                              var simulated_range_dt: Double,
                              var simulated_range_time: Double,
                              var simulated_range_time_old: Double,
                              var simulated_range_CenterNEU: Matrix<Double>,
                              var simulated_range_radius: Double,
                              var simulated_range_ground_speed: Double,
                              var Circumference_Offset_rads: Double,
                              var simulated_range_Two: Double,
                              var simulated_range_total_dt_Two: Double,
                              var simulated_range_dt_Two: Double,
                              var simulated_range_time_Two: Double,
                              var simulated_range_time_old_Two: Double,
                              var simulated_range_CenterNEU_Two: Matrix<Double>,
                              var simulated_range_radius_Two: Double,
                              var simulated_range_ground_speed_Two: Double,
                              var pix2_alt: Double,
                              var pix2_alt_time:Double,
                              var heading_time: Double,
                              var heading: Double,
                              var Image_dt_velocityXYZ: Matrix<Double>,
                              var Image_time_dt_VxVyVz: Matrix<Double>,
                              var Image_velocity_time: Double,
                              var Image_TimeRPY_delayed: Matrix<Double>,
                              var Image_TimeRPY_delayed_1: Matrix<Double>,
                              var Image_TimeRPY_delayed_2: Matrix<Double>,
                              var Image_TimeRPY_delayed_3: Matrix<Double>,
                              var Image_TimeRPY_current: Matrix<Double>,
                              var GPS_Linux_time: Double, //This is not GPS time just linux time the GPS was saved
                              var GPS_lat: Double,
                              var GPS_lon: Double,
                              var GPS_ground_speed: Double,
                              var GPS_height_agl: Double,
                              var GPS_origin_lat: Double,
                              var GPS_origin_lon: Double,
                              var GPS_origin_alt: Double,
                              var GPS_origin_received: Boolean){



}
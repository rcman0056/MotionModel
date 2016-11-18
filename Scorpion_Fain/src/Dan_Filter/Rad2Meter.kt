import golem.mat
import golem.matrix.Matrix

/**
 * Created by ucav on 10/19/2016.
 */


fun Rad2Meter(diff: Matrix<Double>,lat_factor:Double,lon_factor:Double):Matrix<Double>{
    var out= mat[diff[0]*lat_factor,diff[1]*lon_factor,-diff[2] ].T

    return out
}
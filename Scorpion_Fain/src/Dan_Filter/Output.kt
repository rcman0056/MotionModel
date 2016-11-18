import golem.eye
import golem.get
import golem.matrix.Matrix
import golem.set
import golem.zeros
import java.util.*

/**
 * Created by ucav on 9/9/2016.
 */

// this data class was built to make it simpler to access the data rather than having one large matrix.
// It places each component of the output for a given time it's own label
data class Output(var time: Double, var positionLeader: Matrix<Double>, var velocityLeader: Matrix<Double>,
                  var rpyLeader: Matrix<Double>, var covarianceLeader: Matrix<Double>,
                  var positionReceiver: Matrix<Double>, var velocityReceiver: Matrix<Double>, var rpyReceiver: Matrix<Double>,
                  var covarianceReceiver: Matrix<Double>, var P: Matrix<Double>, var xBias: Matrix<Double>, var xBiasCov: Matrix<Double>,
                  var yBias: Matrix<Double>, var yBiasCov: Matrix<Double>,var zBias: Matrix<Double>, var zBiasCov: Matrix<Double>)

//These functions takes an ArrayList of the Output class type and return the specifically requested type as a matrix
fun getTime(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 1)
    value[0,0]=results[0].time

    for(i in 1..size-1) {
        value[i,0]=results[i].time
    }
    return value
}

fun getPositionLeader(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 3)
    value[0,0..2]=results[0].positionLeader

    for(i in 1..size-1) {
        value[i,0..2]=results[i].positionLeader
    }

    return value
}

fun getVelocityLeader(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 3)
    value[0,1..2]=results[0].velocityLeader

    for(i in 1..size-1) {
        value[i,0..2]=results[i].velocityLeader
    }

    return value
}

fun getRpyLeader(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 3)
    value[0,0..2]=results[0].rpyLeader

    for(i in 1..size-1) {
        value[i,0..2]=results[i].rpyLeader
    }

    return value
}

fun getCovarianceLeader(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 15)
    value[0,0..14]=results[0].covarianceLeader

    for(i in 1..size-1) {
        value[i,0..14]=results[i].covarianceLeader
    }

    return value
}

fun getPositionReceiver(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 3)
    value[0,0..2]=results[0].positionReceiver

    for(i in 1..size-1) {
        value[i,0..2]=results[i].positionReceiver
    }

    return value
}

fun getVelocityReceiver(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 3)
    value[0,0..2]=results[0].velocityReceiver

    for(i in 1..size-1) {
        value[i,0..2]=results[i].velocityReceiver
    }

    return value
}

fun getRpyReceiver(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 3)
    value[0,0..2]=results[0].rpyReceiver

    for(i in 1..size-1) {
        value[i,0..2]=results[i].rpyReceiver
    }

    return value
}

fun getCovarianceReceiver(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 15)
    value[0,0..14]=results[0].covarianceReceiver

    for(i in 1..size-1) {
        value[i,0..14]=results[i].covarianceReceiver
    }

    return value
}

//This function calculates and returns the relative covariance through a transformation of the full P matrix
fun getRelativeCovariance(results: ArrayList<Output>): Matrix<Double> {
    var A= zeros(3, 33)
    A[0..2,0..2]=-eye(3)
    A[0..2,15..17]= eye(3)
    var size = results.size
    var output= zeros(size, 3)
    var P=results[0].P
    var transformedP=(A*P*A.T).diag()
    output[0,0..2]=transformedP[0,0..2]
    for (i in 1..size-1)
    {
        P=results[i].P
        transformedP=(A*P*A.T).diag()
        output[i,0..2]=transformedP[0,0..2]
    }

    return output
}

fun getBiasX(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 1)
    value[0,0]=results[0].xBias.getDouble(0)

    for(i in 1..size-1) {
        value[i,0]=results[i].xBias.getDouble(0)
    }

    return value
}

fun getBiasY(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 1)
    value[0,0]=results[0].yBias.getDouble(0)

    for(i in 1..size-1) {
        value[i,0]=results[i].yBias.getDouble(0)
    }

    return value
}

fun getBiasZ(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 1)
    value[0,0]=results[0].zBias.getDouble(0)

    for(i in 1..size-1) {
        value[i,0]=results[i].zBias.getDouble(0)
    }

    return value
}

fun getBiasCovX(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 1)
    value[0,0]=results[0].xBiasCov.getDouble(0)

    for(i in 1..size-1) {
        value[i,0]=results[i].xBiasCov.getDouble(0)
    }

    return value
}
fun getBiasCovY(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 1)
    value[0,0]=results[0].yBiasCov.getDouble(0)

    for(i in 1..size-1) {
        value[i,0]=results[i].yBiasCov.getDouble(0)
    }

    return value
}
fun getBiasCovZ(results: ArrayList<Output>): Matrix<Double> {

    var size=results.size
    var value= zeros(size, 1)
    value[0,0]=results[0].zBiasCov.getDouble(0)

    for(i in 1..size-1) {
        value[i,0]=results[i].zBiasCov.getDouble(0)
    }

    return value
}
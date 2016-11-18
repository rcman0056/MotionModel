import golem.*
import golem.matrix.Matrix
import navutils.rpyToDcm
import navutils.tiltToRpy

/**
 * Adapted to Kotlin Capt Daniel Johnson 9/14/2016.
 */

/*TiltToRpyWithCov  Convert from a DCM with accompanying roll-pitch-yaw Euler
                  angles and roll-pitch-yaw covariance to tilt errors and tilt
                  error covariance

   [tilt_vec, Ptilt] = RpyWithCovToTilt(Dcm, Rpy, PRpy) calculates x, y, and
   z tilt errors and their accompanying covariance about a direction
   cosine matrix when given the tilt error correct roll, pitch, and yaw Euler
   angles and a roll, pitch, and yaw covariance matrix are known. Roll, pitch
   and yaw, are defined as right handed roations about the x, y, and z axes,
   respectively. Dcm is a 3-by-3-by-N array and Tilt is a 3-by-N array.

   An example use case is to calculate x, y, and z tilt errors and
   accompanying tilt error covariance matrix when given a direction cosine
   matrix that rotates from the [nose, right wing, down] body frame to the
   [north, east, down] navigation frame and accompanying titlt error corrected
   roll, pitch, and yaw and roll, pitch, and yaw covariance.

   PARAMETERS
       DcmT: 3-by-3 direction cosine matrix corresponding
       with input tilt angles.

       Rpy: 3-by-1 array of [roll; pitch; yaw]
       expressed in radians. Roll, pitch and yaw, are defined as right handed
       rotations about the x, y, and z axes, respectively.

       PRpy: 3-by-3 covariance matrix of the roll-pitch-yaw errors in
       radians^2.

   RETURN
       tilt_vec: 3-by-N array of [x (rad), y (rad), z (rad)] tilt errors

       Ptilt: 3-by-3 covariance matrix of the tilt errors in radians^2

   REFERENCE
   AUTHOR
      John Raquet
      Autonomy and Navigation Technology (ANT) Center
      Air Force Institute of Technology

      Adapted to Kotlin by Capt Daniel T Johnson Sep 16
      ENG Air Force Institute of Technology
      */

fun RpyWithCovToTilt(Dcm: Matrix<Double>, Rpy: Matrix<Double>, PRpy: Matrix<Double>):tiltData{
    var tilt_vec= zeros(1,3)
    var Ptilt= zeros(3,3)

    var weights=CalculateUkFWeights(null,null,null,null)//Need feedback on how to better code this
    var SigmaPoints=GenerateUkfSigmaPoints(Rpy,PRpy,weights.scaleparam)

    var size=SigmaPoints.numCols()
    var SigmaProp=zeros(3,size)
    for(j in 0..size-1){
        SigmaProp[0..2,j]=calc_tilt(Dcm,SigmaPoints[0..2,j])
    }

    var tiltInfo=UkfMeanCov(SigmaProp,weights.W0m,weights.W0c,weights.Wukf)

    return tiltInfo
}

fun UkfMeanCov(Sigma: Matrix<Double>, W0m: Double, W0c: Double, Wukf: Double): tiltData {
    var NUKFSTATES = Sigma.numRows()
    var NSIGMA=Sigma.numCols()

    var meanUkfCalc=zeros(NUKFSTATES,1)
    var covUkfCalc=zeros(NUKFSTATES,NUKFSTATES)

    meanUkfCalc=W0m*Sigma[0..NUKFSTATES-1,0]+Wukf*sumCols(Sigma[0..NUKFSTATES-1,1..NSIGMA-1])
    var dSigma= Sigma-meanUkfCalc*ones(1,NSIGMA)
    covUkfCalc=W0c*dSigma[0..NUKFSTATES-1,0]*dSigma[0..NUKFSTATES-1,0].T+Wukf*dSigma[0..NUKFSTATES-1,1..NSIGMA-1]*dSigma[0..NUKFSTATES-1,1..NSIGMA-1].T

    return tiltData(meanUkfCalc,covUkfCalc)
}

fun sumCols(matrix:Matrix<Double>):Matrix<Double>{
    var rows=matrix.numRows()
    var cols=matrix.numCols()
    var sumCol=zeros(rows,1)

    for(i in 0..rows-1){
        sumCol[i]=matrix[i,0..cols-1].cumSum().get(cols-1)
    }

    return sumCol
}

fun CalculateUkFWeights(NUKFSTATESIn:Int?,alphaIn:Double?,betaIn:Double?,kappaIn:Double?):UkfWeights{
    /* CalculateUkfWeights   Calculates the UKF weighting parameters based on the desired
                                   scaling parameters.

     [W0m, W0c, Wukf, scaleparam] = CalculateUkfWeights(NSTATES, alpha, beta, kappa)

     NSTATES:  Number of states.
     alpha:    Sigma point spread parameter (optional, DEFAULT = 1e-3)
     beta:     Prior knowledge parameter (optional, DEFAULT = 2, i.e., Gaussian dist)
     kappa:    Secondary scaling parameter (optional, DEFAULT = 0)
     W0m, W0c, Wukf:   UKF weighting parameters
     scaleparam:   Scaling parameter:  scaleparam = NSTATES + lambda

     Michael J. Veth, Dec 2009
     Adapted to Kotlin by Capt Daniel T Johnson Sep 16
    */
    var NUKFSTATES=3
    var alpha=.001
    var beta=2
    var kappa=0

    if(NUKFSTATESIn!=null) {
        var NUKFSTATES = NUKFSTATESIn
    }
    if(alphaIn!=null){
        var alpha=alphaIn
    }
    if(betaIn!=null){
        var beta=betaIn
    }
    if(kappaIn!=null){
        var kappa=kappaIn
    }

    var lambda= pow(alpha, 2) *(NUKFSTATES+kappa)-NUKFSTATES
    var scaleparam=NUKFSTATES+lambda
    var W0m=lambda/scaleparam
    var W0c=W0m+(1- pow(alpha, 2) +beta)

    var Wukf=1.0/2.0/scaleparam


    return UkfWeights(scaleparam,W0m,W0c,Wukf)
}

fun GenerateUkfSigmaPoints(x_hat: Matrix<Double>, Pxx: Matrix<Double>, scaleparam: Double):Matrix<Double>{
    /*
     GenerateUkfSigmaPoints    Converts a mean and covariance into a
                               collection of UKF sigma points.
     SigmaPoints = GenerateUkfSigmaPoints(xhat, Pxx, scaleparameter)

     PARAMETERS
               x_hat: Nx1 vector to generate points around

               Pxx: NxN matrix of covariances to spread sigma points

               scaleparameter: is the UKF scale parameter (sqrt(lambda + L))

     RETURN
               SigmaPoints is an Nx(2N+1) matrix of the generated sigma points


 Michael J. Veth, November 2009
 Adapted to Kotlin by Capt Daniel T Johnson Sep 16
     */
    var NSTATES=x_hat.asColVector().numRows()
    var cholPxx = sqrt(scaleparam)*Pxx.chol().T
    var SigmaPoints= zeros(NSTATES,2*NSTATES+1)

    SigmaPoints[0..NSTATES-1,0..0]= x_hat.asColVector()
    SigmaPoints[0..NSTATES-1,1..NSTATES]=x_hat*ones(1,NSTATES)+cholPxx
    SigmaPoints[0..NSTATES-1,NSTATES+1..2*NSTATES]=x_hat*ones(1,NSTATES)-cholPxx

    return SigmaPoints
}

fun calc_tilt(Dcm:Matrix<Double>,Rpy:Matrix<Double>):Matrix<Double>{
    /*
    calc_rpy  Calculate tilt error from tilt error corrected roll-pitch-yaw Euler
          angles about a given DCM

    tilt = calc_tilt(Cnb, rpy) Calculate tilt error from tilt error corrected
    roll-pitch-yaw Euler angles about a given DCM

    An example use case is to calculate x, y, and z tilt errors when given a
    direction cosine matrix that rotates from the [nose, right wing, down] body
    frame to the [north, east, down] navigation frame and
    accompanying tilt error correct roll, pitch, and yaw.

    PARAMETERS
       Dcm: 3-by-3 direction cosine matrix corresponding
       with input tilt angles.

       Rpy: 3-by-1 vector of [roll; pitch; yaw]
       expressed in radians. Roll, pitch and yaw, are defined as right handed
       rotations about the x, y, and z axes, respectively.

    RETURN
       tilt_vec: 3-by-1 vector of [x (rad), y (rad), z (rad)] tilt errors

    REFERENCE

    AUTHOR
      John Raquet
      Autonomy and Navigation Technology (ANT) Center
      Air Force Institute of Technology

      Adapted to Kotlin by Capt Daniel T Johnson Sep 16
     */

    var Cnb_estimate=rpyToDcm(Rpy).T

    var expmPsy=Cnb_estimate*Dcm.T
    var Psy=expmPsy //Need to do logm function here.
    var tilt_vec=zeros(3,1)
    tilt_vec[0]=Psy[2,1]
    tilt_vec[1]=-Psy[2,0]
    tilt_vec[2]=Psy[1,0]

    return(tilt_vec)
}

fun logm(expm:Matrix<Double>):Matrix<Double>{
    var trace=expm.diag().sum()
    var theta=asin((trace-1.0)/2.0)
    var log=theta/(2.0*sin(theta))*(expm*expm.T) //Need to do logm function here
    return(log)
}

data class UkfWeights(var scaleparam:Double, var W0m:Double,var W0c:Double,var Wukf:Double)

data class tiltData(var tilt_vec: Matrix<Double>, var Ptilt: Matrix<Double>)
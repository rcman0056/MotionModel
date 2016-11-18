
import golem.*
import golem.matrix.Matrix
import navutils.rpyToDcm
import java.util.*

/**
 * Adapted by Capt Daniel Johnson 9/14/2016.
 */

fun StereoRpyWithCovToTilt(Cbn_T: Matrix<Double>,Cbn_R: Matrix<Double>, Rpy: Matrix<Double>, PRpy: Matrix<Double>):tiltData{
    var tilt_vec= zeros(1,3)
    var Ptilt= zeros(3,3)

    var weights=CalculateUkFWeights(null,null,null,null)//Need feedback on how to better code this
    var SigmaPoints=GenerateUkfSigmaPoints(Rpy,PRpy,weights.scaleparam)

    var size=SigmaPoints.numCols()
    var SigmaProp=zeros(3,size)
    for(j in 0..size-1){
        SigmaProp[0..2,j]=calc_tilt_stereo(Cbn_T,Cbn_R,SigmaPoints[0..2,j])
    }

    var tiltInfo=UkfMeanCov(SigmaProp,weights.W0m,weights.W0c,weights.Wukf)

    return tiltInfo
}

fun calc_tilt_stereo(Cbn_T: Matrix<Double>, Cbn_R: Matrix<Double>, Rpy: Matrix<Double>):Matrix<Double> {
    var Ctr_meas=rpyToDcm(Rpy).T

    var expmPsy=Cbn_T.T*Ctr_meas*Cbn_R
    var Psy=logm(expmPsy) //Need to do logm function here.

    var tilt_vec=zeros(3,1)
    tilt_vec[0]=Psy[2,1]
    tilt_vec[1]=-Psy[2,0]
    tilt_vec[2]=Psy[1,0]

    return tilt_vec
}

import com.jcg.*
import golem.*
import golem.matrix.Matrix
import java.util.*

/**
 * Created by ucav on 9/9/2016.
 */

fun plotResults(results: ArrayList<Output>){


    var gpsTimeTruth = truthL.getCol(0)
    var plotTimeTruth=gpsTimeTruth.minus(gpsTimeTruth.get(0))
    var gpsTimeResults = getTime(results)
    var plotTimeResults=gpsTimeResults.minus(gpsTimeResults.get(0))

    var t=plotTimeResults.numRows();
    var resultRows=results.size
    //var resultCols=results.numCols()
    var truthSize= truthL.numRows()
    plotTimeResults=plotTimeResults[0..truthSize-1,0]
    //var shortResults=results[0..truthSize-1]

    var trueLatL= truthL.getCol(1).times(dtr)
    var trueLonL= truthL.getCol(2).times(dtr)
    var trueAltL= truthL.getCol(3)

    var trueLatR= truthR.getCol(1).times(dtr)
    var trueLonR= truthR.getCol(2).times(dtr)
    var trueAltR= truthR.getCol(3)

    var posL= getPositionLeader(results)
    var velL= getVelocityLeader(results)
    var rpyL= getRpyLeader(results)
    var covL= getCovarianceLeader(results)

    var posR= getPositionReceiver(results)
    var velR= getVelocityReceiver(results)
    var rpyR= getRpyReceiver(results)
    var covR= getCovarianceReceiver(results)

    var relCov= getRelativeCovariance(results)



    var estLatL=posL.get(0..truthSize-1,0)
    var estLonL=posL.get(0..truthSize-1,1)
    var estAltL=posL.get(0..truthSize-1,2)

    var estLatR=posR.get(0..truthSize-1,0)
    var estLonR=posR.get(0..truthSize-1,1)
    var estAltR=posR.get(0..truthSize-1,2)

    var trueLatDiff=(trueLatR-trueLatL).times(rtm.second)
    var trueLonDiff=(trueLonR-trueLonL).times(rtm.first)
    var trueAltDiff=(trueAltR-trueAltL)

    var estLatDiff=(estLatR-estLatL).times(rtm.second)
    var estLonDiff=(estLonR-estLonL).times(rtm.first)
    var estAltDiff=(estAltR-estAltL)

    var errLat=estLatDiff-trueLatDiff
    var errLon=estLonDiff-trueLonDiff
    var errAlt=estAltDiff-trueAltDiff

    var nCovR=covR.get(0..truthSize-1,0)
    var eCovR=covR.get(0..truthSize-1,1)
    var dCovR=covR.get(0..truthSize-1,2)

    var nCovL=covL.get(0..truthSize-1,0)
    var eCovL=covL.get(0..truthSize-1,1)
    var dCovL=covL.get(0..truthSize-1,2)


    var latDiffCov=relCov.get(0..truthSize-1,0)
    var lonDiffCov=relCov.get(0..truthSize-1,1)
    var altDiffCov=relCov.get(0..truthSize-1,2)

    var zBias=getBiasZ(results)
    var zBiasCov=getBiasCovZ(results)


    var plotCount=1
    var onePlot=true
    //plotCount=plotIndividualAircraftPositionError(plotTimeResults,plotCount,posL,posR)
    //plotCount=plotPositionDiff(plotTimeResults,plotCount,trueLatDiff,trueLonDiff,trueAltDiff,"Truth",onePlot)
    //plotCount=plotPositionDiff(plotTimeResults,plotCount,estLatDiff,estLonDiff,estAltDiff,"Estimated",onePlot)
    plotCount=plotPositionDiff(plotTimeResults,plotCount,errLat,errLon,errAlt,"Error",onePlot)
    plotCount=plotPositionDiffCov(plotTimeResults,plotCount,errLat,errLon,errAlt,"Error", sqrt(latDiffCov), sqrt(lonDiffCov), sqrt(altDiffCov))
    plotCount=plotDiffCov(plotTimeResults,plotCount,zBias,"Error",sqrt(zBiasCov))
    //plotCount=plotAltDiff(plotTimeResults,plotCount,posL,posR)
    //plotCount=plotIndividualAircraftRPY(plotTimeResults,plotCount,rpyL,rpyR)
}
fun plotIndividualAircraftRPY(plotTime: Matrix<Double>, counter:Int, rpyL:Matrix<Double>,rpyR:Matrix<Double>):Int{
    var count=counter

    figure(count++)
    plot(plotTime,rpyL.getCol(1).times(180).div(Math.PI),"b","Leader Pitch")
    plot(plotTime,rpyR.getCol(1).times(180).div(Math.PI),"r","Receiver Pitch")

    return count
}

fun plotAltDiff(plotTime: Matrix<Double>, counter:Int, PosL: Matrix<Double>, PosR: Matrix<Double>):Int{
    var count=counter
    figure(count++)
    plot(plotTime, PosL.getCol(2), "b", "Tanker Est Altitude")
    plot(plotTime, PosR.getCol(2), "r", "Receiver Est Altitude")

    figure(count++)
    plot(plotTime, truthL.getCol(3), "b", "Tanker Est Altitude")
    plot(plotTime, truthR.getCol(3), "r", "Receiver Est Altitude")

    return count
}

fun plotIndividualAircraftPositionError(plotTime: Matrix<Double>, counter:Int, PosL: Matrix<Double>, PosR: Matrix<Double>):Int{
    var count=counter
    figure(count++)
    plot(plotTime, (PosL.getCol(0).minus(truthL.getCol(1).times(dtr))).times(rtm.second), "b", "Tanker Lat Error (m)")
    plot(plotTime, (PosL.getCol(1).minus(truthL.getCol(2).times(dtr))).times(rtm.first), "r", "Tanker Lon Error (m)")
    plot(plotTime, (PosL.getCol(2).minus(truthL.getCol(3))), "y", "Tanker Altitude Error (m)")
    xlabel("Time (s)")
    ylabel("Error (m)")
    title("Tanker Error")

    figure(count++)
    plot(plotTime, (PosR.getCol(0).minus(truthR.getCol(1).times(dtr))).times(rtm.second), "b", "Receiver Lat Error (m)")
    plot(plotTime, (PosR.getCol(1).minus(truthR.getCol(2).times(dtr))).times(rtm.first), "r", "Receiver Lon Error (m)")
    plot(plotTime, (PosR.getCol(2).minus(truthR.getCol(3))), "y", "Receiver Altitude Error (m)")
    xlabel("Time (s)")
    ylabel("Error (m)")
    title("Receiver Error")
    return count
}

fun plotPositionDiffCov(plotTime: Matrix<Double>, counter:Int, LatDiff: Matrix<Double>, LonDiff: Matrix<Double>, AltDiff: Matrix<Double>, label:String, latDiffCov: Matrix<Double>, lonDiffCov: Matrix<Double>, altDiffCov: Matrix<Double>):Int{
    var count=counter

    var plotCov= vstack(backwardsVector(latDiffCov), -latDiffCov)
    var time= vstack(backwardsVector(plotTime), plotTime)
    figure(count++)
    plot(plotTime, LatDiff, "b", "Relative Lat " + label)
    plot(time, plotCov, "r", "1-sigma uncertainty")

    ylabel("Error (m)")
    xlabel("Time (s)")
    title("Relative Latitude " + label)

    plotCov= vstack(backwardsVector(lonDiffCov), -lonDiffCov)
    figure(count++)
    plot(plotTime, LonDiff, "b", "Relative Lon " + label)
    plot(time, plotCov, "r", "1-sigma uncertainty")
    ylabel("Error (m)")
    xlabel("Time (s)")
    title("Relative Longitude " + label)

    plotCov= vstack(backwardsVector(altDiffCov), -altDiffCov)

    figure(count++)
    plot(plotTime, AltDiff, "b", "Relative Alt " + label)
    plot(time, plotCov, "r", "1-sigma uncertainty")
    ylabel("Error (m)")
    xlabel("Time (s)")
    title("Relative Altitude " + label)
    return count
}

fun plotDiffCov(plotTime: Matrix<Double>, counter:Int, LatDiff: Matrix<Double>, label:String, latDiffCov: Matrix<Double>):Int{
    var count=counter

    var plotCov= vstack(backwardsVector(latDiffCov), -latDiffCov)
    var time= vstack(backwardsVector(plotTime), plotTime)
    figure(count++)
    plot(plotTime, LatDiff, "b", "Error " + label)
    plot(time, plotCov, "r", "1-sigma uncertainty")

    ylabel("Error (m)")
    xlabel("Time (s)")
    title("Bias " + label)

    return count
}

fun plotPositionDiff(plotTime: Matrix<Double>, counter:Int, LatDiff: Matrix<Double>, LonDiff: Matrix<Double>, AltDiff: Matrix<Double>, label:String, onePlot:Boolean):Int{
    //plot the true relative position and the calculated relative position
    var count=counter
    figure(count++)
    if(onePlot==false){
        plot(plotTime, LatDiff, "b", "Lat Difference " + label)
        ylabel("Error (m)")
        xlabel("Time (s)")
        title("Latitude Difference " + label)

        figure(count++)
        plot(plotTime, LonDiff, "r", "Lon Difference" + label)
        ylabel("Error (m)")
        xlabel("Time (s)")
        title("Longitude Difference " + label)

        figure(count++)
        plot(plotTime, AltDiff, "y", "Alt Difference" + label)
        ylabel("Error (m)")
        xlabel("Time (s)")
        title("Altitude Difference " + label)
    }
    else{
        plot(plotTime, LatDiff, "b", "Lat Difference " + label)
        plot(plotTime, LonDiff, "r", "Lon Difference " + label)
        plot(plotTime, AltDiff, "y", "Alt Difference " + label)
        ylabel("Error (m)")
        xlabel("Time (s)")
        title("Relative Position " + label)
    }
    return count
}

fun backwardsVector(vector:Matrix<Double>):Matrix<Double>{
    var size=vector.numRows()
    var backwards=zeros(size,1)
    for(i in 0..size-1)
    {
        backwards[size-1-i]=vector[i]
    }

    return backwards
}

fun posCov(cov:Matrix<Double>):Matrix<Double>{
    var size=cov.numRows()
    var A=zeros(3,30)
    A[0..2,0..2]=-eye(3)
    A[0..2,14..16]=eye(3)
    for(i in 0..size-1)
    {

    }
    return cov
}
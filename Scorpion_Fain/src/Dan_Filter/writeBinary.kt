/**
 * Created by ucav on 9/9/2016.
 */

import Output
import golem.asRowVector
import golem.hstack
import golem.matrix.Matrix
import java.io.BufferedOutputStream
import java.io.DataOutputStream
import java.io.File
import java.io.FileOutputStream
import java.util.*


fun writeArrayToBinary(truth: Matrix<Double>, fileName:String){
    var size=truth.numRows()

    var out = DataOutputStream(BufferedOutputStream(FileOutputStream(fileName)))
    var count =0
    for(i in 0..size-1){
        var row=truth.getRow(i).asRowVector()
        System.out.print(i)
        for(j in 1..3) {
            out.writeDouble(row[j].toDouble())
            count++
        }
    }
    out.close()
    System.out.print(count)
    var test=1
}

fun writeToFileBinary(results: ArrayList<Output>, fileName:String){
        var size = results.size

    var out = DataOutputStream(BufferedOutputStream(FileOutputStream(fileName)))

    for(i in 0..size-1){
        var row=results[i]
        writeTimeBinary(out,row)
        writePositionLeaderBinary(out,row)
        writeVelocityLeaderBinary(out,row)
        writeRpyLeaderBinary(out,row)
        writeCovarianceLeaderBinary(out,row)
        writePositionReceiverBinary(out,row)
        writeVelocityReceiverBinary(out,row)
        writeRpyReceiverBinary(out,row)
        writeCovarianceReceiverBinary(out,row)
        writePBinary(out,row)
        writexBiasBinary(out,row)
        writexBiasCovBinary(out,row)
        writeyBiasBinary(out,row)
        writeyBiasCovBinary(out,row)
        writezBiasBinary(out,row)
        writezBiasCovBinary(out,row)

    }
    out.close()
}

fun writeTimeBinary(out:DataOutputStream, row: Output){
    var tempData=row.time
    var a=tempData.toDouble()
    out.writeDouble(tempData.toDouble())
}

fun writePositionLeaderBinary(out:DataOutputStream, row: Output) {
    var tempData=row.positionLeader.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}
fun writeVelocityLeaderBinary(out:DataOutputStream, row: Output) {
    var tempData=row.velocityLeader.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

fun writeRpyLeaderBinary(out:DataOutputStream, row: Output) {
    var tempData=row.rpyLeader.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

fun writeCovarianceLeaderBinary(out:DataOutputStream, row: Output) {
    var tempData=row.covarianceLeader.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

fun writePositionReceiverBinary(out:DataOutputStream, row: Output) {
    var tempData=row.positionReceiver.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}
fun writeVelocityReceiverBinary(out:DataOutputStream, row: Output) {
    var tempData=row.velocityReceiver.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

fun writeRpyReceiverBinary(out:DataOutputStream, row: Output) {
    var tempData=row.rpyReceiver.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

fun writeCovarianceReceiverBinary(out:DataOutputStream, row: Output) {
    var tempData=row.covarianceReceiver.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

fun writePBinary(out:DataOutputStream, row: Output){
    var P=row.P
    var rowVec=P.getRow(0)
    for (i in 1..rowVec.numCols()-1){
        rowVec = hstack(rowVec, P.getRow(i))
    }
    var tempData=rowVec
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

fun writexBiasBinary(out:DataOutputStream, row: Output) {
    var tempData=row.xBias.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

fun writexBiasCovBinary(out:DataOutputStream, row: Output) {
    var tempData=row.xBiasCov.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

fun writeyBiasBinary(out:DataOutputStream, row: Output) {
    var tempData=row.yBias.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

fun writeyBiasCovBinary(out:DataOutputStream, row: Output) {
    var tempData=row.yBiasCov.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

fun writezBiasBinary(out:DataOutputStream, row: Output) {
    var tempData=row.zBias.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

fun writezBiasCovBinary(out:DataOutputStream, row: Output) {
    var tempData=row.zBiasCov.asRowVector()
    var size = tempData.numCols()
    for(i in 0..size-1)
        out.writeDouble(tempData[i].toDouble())
}

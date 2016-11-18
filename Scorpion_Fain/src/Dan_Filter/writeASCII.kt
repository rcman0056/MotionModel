import Output
import golem.asRowVector
import golem.hstack
import java.io.File
import java.util.*

/**
 * Created by ucav on 9/9/2016.
 */

fun writeToFileASCII(results: ArrayList<Output>, fileName:String){
    val write= File(fileName)
    var size = results.size

    write.writeText("Time\t")
    write.appendText("positionLeader\tvelocityLeader\trpyLeader\tcovarianceLeader\t")
    write.appendText("positionReceiver\tvelocityReceiver\trpyReceiver\tcovarianceReceiver\t")
    write.appendText("P\n")
    for(i in 0..size-1){
        var row=results[i]
        writeTimeASCII(write,row)
        writePositionLeaderASCII(write,row)
        writeVelocityLeaderASCII(write,row)
        writeRpyLeaderASCII(write,row)
        writeCovarianceLeaderASCII(write,row)
        writePositionReceiverASCII(write,row)
        writeVelocityReceiverASCII(write,row)
        writeRpyReceiverASCII(write,row)
        writeCovarianceReceiverASCII(write,row)
        writePASCII(write,row)
        write.appendText("\n")
    }
}


fun writeTimeASCII(fileName: File, row: Output){
    var tempData=row.time.toString()
    fileName.appendText("$tempData\t")
}

fun writePositionLeaderASCII(fileName: File, row: Output) {
    var tempData=row.positionLeader.asRowVector().toString()
    tempData=tempData.replace(" ","")
    tempData=tempData.replace("]","")
    tempData=tempData.replace(",","\t")
    tempData=tempData.removeRange(0..3)
    fileName.appendText("$tempData\t")
}
fun writeVelocityLeaderASCII(fileName: File, row: Output) {
    var tempData=row.velocityLeader.asRowVector().toString()
    tempData=tempData.replace(" ","")
    tempData=tempData.replace("]","")
    tempData=tempData.replace(",","\t")
    tempData=tempData.removeRange(0..3)
    fileName.appendText("$tempData\t")
}

fun writeRpyLeaderASCII(fileName: File, row: Output) {
    var tempData=row.rpyLeader.asRowVector().toString()
    tempData=tempData.replace(" ","")
    tempData=tempData.replace("]","")
    tempData=tempData.replace(",","\t")
    tempData=tempData.removeRange(0..3)
    fileName.appendText("$tempData\t")
}

fun writeCovarianceLeaderASCII(fileName: File, row: Output) {
    var tempData=row.covarianceLeader.asRowVector().toString()
    tempData=tempData.replace(" ","")
    tempData=tempData.replace("]","")
    tempData=tempData.replace(",","\t")
    tempData=tempData.removeRange(0..3)
    fileName.appendText("$tempData\t")
}

fun writePositionReceiverASCII(fileName: File, row: Output) {
    var tempData=row.positionReceiver.asRowVector().toString()
    tempData=tempData.replace(" ","")
    tempData=tempData.replace("]","")
    tempData=tempData.replace(",","\t")
    tempData=tempData.removeRange(0..3)
    fileName.appendText("$tempData\t")
}
fun writeVelocityReceiverASCII(fileName: File, row: Output) {
    var tempData=row.velocityReceiver.asRowVector().toString()
    tempData=tempData.replace(" ","")
    tempData=tempData.replace("]","")
    tempData=tempData.replace(",","\t")
    tempData=tempData.removeRange(0..3)
    fileName.appendText("$tempData\t")
}

fun writeRpyReceiverASCII(fileName: File, row: Output) {
    var tempData=row.rpyReceiver.asRowVector().toString()
    tempData=tempData.replace(" ","")
    tempData=tempData.replace("]","")
    tempData=tempData.replace(",","\t")
    tempData=tempData.removeRange(0..3)
    fileName.appendText("$tempData\t")
}

fun writeCovarianceReceiverASCII(fileName: File, row: Output) {
    var tempData=row.covarianceReceiver.asRowVector().toString()
    tempData=tempData.replace(" ","")
    tempData=tempData.replace("]","")
    tempData=tempData.replace(",","\t")
    tempData=tempData.removeRange(0..3)
    fileName.appendText("$tempData\t")
}

fun writePASCII(fileName: File, row: Output){
    var P=row.P
    var rowVec=P.getRow(0)
    for (i in 1..29){
        rowVec = hstack(rowVec, P.getRow(i))
    }
    var tempData=rowVec.toString()
    tempData=tempData.replace(" ","")
    tempData=tempData.replace("]","")
    tempData=tempData.replace(",","\t")
    tempData=tempData.removeRange(0..3)
    fileName.appendText("$tempData\t")

}
fun Append(fileName: File, tempData:String){}
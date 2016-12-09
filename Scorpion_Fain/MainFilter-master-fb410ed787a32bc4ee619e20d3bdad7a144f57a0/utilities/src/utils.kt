import golem.matrix.Matrix
import golem.zeros
import java.io.*
import java.util.*
import java.nio.ByteBuffer

//Written by Capt Daniel Johnson
fun readTdf(fileName: String, DELIMITER: String, dim: IntArray): ArrayList<Matrix<Double>> {
    val numCols = dim[0]
    val numRows = dim[1]-dim[2]
    val start   = dim[2]

    var fileReader = BufferedReader(FileReader(fileName))
    var data: ArrayList<Matrix<Double>> = ArrayList()

    var row = zeros(numCols)

    //If there is no header do the first line, otherwise skip
    var line = fileReader.readLine()
    for (i in 1..start) {
        line = fileReader.readLine()
    }
    var tokens = line.split(DELIMITER)

        for (i in 0..numCols - 1) {
            row[i] = tokens[i].toDouble()
        }
    data.add(row)

    var count = 0;
    var done = false
    while ( { line = fileReader.readLine() ; line }() != null && !done) {
        row = zeros(numCols)
        tokens = line.split(DELIMITER)
        for (i in 0..numCols-1) {
            row[i] = tokens[i].toDouble()
        }


        if(numRows>0) {
            count++
            if (count==numRows){
                done = true
            }
            else {
                data.add(row)
            }
        }
    }
    return data
}

//Written by Capt Daniel Johnson
fun convertArrayListToArray(list: ArrayList<Matrix<Double>>): Matrix<Double> {

    var rows = list.size
    var cols = list.get(0).numCols()
    var b = zeros(rows, cols)
    for (i in 0..rows - 1) {
        var row = list[i]
        for (j in 0..cols - 1) {
            b[i, j] = row.get(j)
        }
    }
    return b
}

fun writeToFileASCII(results: Matrix<Double>, fileName: String) {
    val file = File(fileName)

    file.writeText("")
    val col = results.numCols()
    val row = results.numRows()

    for (i in 0..row - 1) {
        for (j in 0..col - 1) {
            file.appendText("${results.get(i, j)}\t")
        }
        file.appendText("\n")
        val progress = (i.toDouble()+1)/row.toDouble()*100
        System.out.println(progress.toString()+"% done writing results")
    }
}


fun writeToFileBinary(results: Matrix<Double>, fileName:String){
    System.out.println("Writing results to binary file...")
    var doubleArray = DoubleArray(results.numRows()*results.numCols())
    for(i in 0..results.numRows()-1){
        for(j in 0..results.numCols()-1){
            doubleArray.set(j+results.numCols()*i,results.get(i,j))
        }
    }
    val byteArray = toByteArray(doubleArray)
    val fileTarget = File(fileName)
    val fileTemp = File("tempFile")
    fileTemp.writeBytes(byteArray)
    fileTemp.copyTo(fileTarget,true)
    System.out.println("...results written")
}

fun toByteArray(doubleArray: DoubleArray): ByteArray{
    val times = 8 //doubleSize/byteSize
    var bytes = ByteArray(doubleArray.size * times)
    for(i in 0..doubleArray.size-1){
        ByteBuffer.wrap(bytes, i*times, times).putDouble(doubleArray[i])
    }
    return bytes
}

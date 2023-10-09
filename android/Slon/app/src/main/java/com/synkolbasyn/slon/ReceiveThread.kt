package com.synkolbasyn.slon

import android.bluetooth.BluetoothSocket
import android.util.Log
import androidx.compose.runtime.mutableStateOf
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream

val lat = mutableStateOf(0.0)
val lon = mutableStateOf(0.0)

class ReceiveThread(socket: BluetoothSocket) : Thread() {
    var iStream: InputStream? = null
    var oStream: OutputStream? = null

    init {
        try {
            iStream = socket.inputStream
        }
        catch (i: IOException) {
            Log.d("MY_ERRORS", "iStream")
        }
        try {
            oStream = socket.outputStream
        }
        catch (i: IOException) {
            Log.d("MY_ERRORS", "oStream")
        }
    }

    override fun run() {
        val buf: ByteArray = ByteArray(1024)
        while (true) {
            try {
                val size = iStream?.read(buf)
                val message = String(buf, 0, size!!)
                Log.d("MY_ERRORS", message)
                val values: List<String> = message.split(";")
                lat.value = values[0].toDouble()
                lon.value = values[1].toDouble()
            }
            catch (i: IOException) {
                break
            }
        }
    }

    fun sendMessage(byteArray: ByteArray) {
        try {
            oStream?.write(byteArray)
            Log.d("MY_ERRORS", byteArray.decodeToString())
        }
        catch (i: IOException) {
            Log.d("MY_ERRORS", "sendMesssage")
        }
    }
}
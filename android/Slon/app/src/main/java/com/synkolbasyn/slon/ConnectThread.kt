package com.synkolbasyn.slon

import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.util.Log
import android.widget.Toast
import java.io.IOException
import java.util.UUID

class ConnectThread(device: BluetoothDevice) : Thread() {
    var socket: BluetoothSocket? = null
    lateinit var rThread: ReceiveThread

    init {
        try {
            Log.d("MY_ERRORS", "start creating sock")
            socket = device.createRfcommSocketToServiceRecord(UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"))
            Log.d("MY_ERRORS", "sock created")
        }
        catch (i: IOException) {
            Log.d("MY_ERRORS", "create sock")
        }
    }

    override fun run() {
        try {
            Log.d("MY_ERRORS", "connecting...")
            socket?.connect()
            Log.d("MY_ERRORS", "Connected")
            rThread = ReceiveThread(socket!!)
            Log.d("MY_ERRORS", "created rthrad")
            rThread.start()
            Log.d("MY_ERRORS", "rthread started")
        }
        catch (i: IOException) {
            Log.d("MY_ERRORS", "RUN CThread")
            Log.d("MY_ERRORS", "$i")
            closeConnection()
        }
    }

    fun closeConnection() {
        try {
            socket?.close()
        }
        catch (i: IOException) {
            Log.d("MY_ERRORS", "Close conn")
        }
    }
}
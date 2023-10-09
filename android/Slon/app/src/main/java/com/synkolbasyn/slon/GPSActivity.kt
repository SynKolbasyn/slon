package com.synkolbasyn.slon

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.wrapContentSize
import androidx.compose.material3.Button
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.synkolbasyn.slon.ui.theme.SlonTheme


lateinit var cThread: ConnectThread
lateinit var bluetoothManager: BluetoothManager
lateinit var bluetoothAdapter: BluetoothAdapter


class GPSActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        bluetoothManager = getSystemService(BluetoothManager::class.java)
        bluetoothAdapter = bluetoothManager.adapter
        Log.d("MY_ERRORS", "${intent.getStringExtra("mac")}")
        Log.d("MY_ERRORS", "${bluetoothAdapter.isEnabled}")
        cThread = ConnectThread(bluetoothAdapter.getRemoteDevice(intent.getStringExtra("mac")))
        cThread.start()

        setContent {
            SlonTheme {
                SlonApp()
            }
        }
    }
}


@Composable
fun SlonApp() {
    GPSActivity(
        modifier = Modifier
            .fillMaxSize()
            .wrapContentSize(Alignment.Center)
    )
}


@Composable
fun GPSActivity(modifier: Modifier) {
    val latitude by lat
    val longitude by lon
    Column (
        modifier = modifier,
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(text = "Latitude: $latitude")
            Spacer(modifier = Modifier.width(10.dp))
            Text(text = "Longitude: $longitude")
        }
        Spacer(modifier = Modifier.height(500.dp))
        Button(onClick = { sendMessage("Hello ESP32") }) {
            Text(text = stringResource(id = R.string.button_save))
        }
    }
}


fun sendMessage(message: String) {
    cThread.rThread.sendMessage(message.toByteArray())
}

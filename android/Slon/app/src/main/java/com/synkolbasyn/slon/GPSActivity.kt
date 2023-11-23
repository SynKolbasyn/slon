package com.synkolbasyn.slon

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.wrapContentSize
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.Button
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.input.key.Key
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.input.ImeAction
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.tooling.preview.Preview
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

@Preview(showBackground = true)
@Composable
fun SlonApp() {
    GPSActivity(
        modifier = Modifier
            .fillMaxSize()
            .wrapContentSize(Alignment.Center)
    )
}


@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun GPSActivity(modifier: Modifier) {
    val latitude by lat
    val longitude by lon
    var speed by remember { mutableStateOf("") }
    var prev_command: String = "Stop"
    Column (
        modifier = Modifier.fillMaxSize(),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.SpaceBetween
    ) {
        Row (
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceEvenly
        ) {
            Text(text = "Latitude: $latitude")
            Text(text = "Longitude: $longitude")
        }
        Column (
            modifier = Modifier.fillMaxWidth(),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.SpaceAround
        ) {
            TextField (
                value = speed,
                onValueChange = {
                    speed = it.trim()
                    Log.d("MY_ERRORS", "TEXT: $speed")
                },
                label = { Text("Speed") },
                keyboardOptions = KeyboardOptions(
                    keyboardType = KeyboardType.Number,
                    imeAction = ImeAction.Done
                ),
                singleLine = true,
                keyboardActions = KeyboardActions(onDone = {sendMessage("$prev_command;$speed")})
            )
            Button(
                onClick = {
                    sendMessage("Forward;$speed")
                    prev_command = "Forward"
                }
            ) {
                Text(
                    text = stringResource(id = R.string.button_forward),
                    textAlign = TextAlign.Center,
                    modifier = Modifier.width(60.dp)
                )
            }
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceEvenly
            ) {
                Button(
                    onClick = {
                        sendMessage("Left;$speed")
                        prev_command = "Left"
                    }
                ) {
                    Text(
                        text = stringResource(id = R.string.button_left),
                        textAlign = TextAlign.Center,
                        modifier = Modifier.width(60.dp)
                    )
                }
                Button(
                    onClick = {
                        sendMessage("Stop;$speed")
                        prev_command = "Stop"
                    }
                ) {
                    Text(
                        text = stringResource(id = R.string.button_stop),
                        textAlign = TextAlign.Center,
                        modifier = Modifier.width(60.dp)
                    )
                }
                Button(
                    onClick = {
                        sendMessage("Right;$speed")
                        prev_command = "Right"
                    }
                ) {
                    Text(
                        text = stringResource(id = R.string.button_right),
                        textAlign = TextAlign.Center,
                        modifier = Modifier.width(60.dp)
                    )
                }
            }
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceEvenly
            ) {
                Button(
                    onClick = {
                        sendMessage("Save;$speed")
                        prev_command = "Stop"
                    }
                ) {
                    Text(
                        text = stringResource(id = R.string.button_save),
                        textAlign = TextAlign.Center,
                        modifier = Modifier.width(60.dp)
                    )
                }
                Button(
                    onClick = {
                        sendMessage("Back;$speed")
                        prev_command = "Back"
                    }
                ) {
                    Text(
                        text = stringResource(id = R.string.button_back),
                        textAlign = TextAlign.Center,
                        modifier = Modifier.width(60.dp)
                    )
                }
                Button(
                    onClick = {
                        sendMessage("Start;$speed")
                        prev_command = "Stop"
                    }
                ) {
                    Text(
                        text = stringResource(id = R.string.button_start),
                        textAlign = TextAlign.Center,
                        modifier = Modifier.width(60.dp)
                    )
                }
            }
        }
    }
}


fun sendMessage(message: String) {
    cThread.rThread.sendMessage(message.toByteArray())
}

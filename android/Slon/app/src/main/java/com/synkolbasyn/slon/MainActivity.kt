package com.synkolbasyn.slon


import android.Manifest
import android.app.Activity
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.ActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.wrapContentSize
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat.startActivity
import com.synkolbasyn.slon.ui.theme.SlonTheme


class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
//
        val requestPermissionLauncher = registerForActivityResult(
            ActivityResultContracts.RequestPermission()
        ) { isGranted: Boolean ->
            if (!isGranted) { this.finish() }
        }

        if (
            ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.BLUETOOTH
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            requestPermissionLauncher.launch(Manifest.permission.BLUETOOTH)
        }

        val bluetoothManager: BluetoothManager = getSystemService(BluetoothManager::class.java)
        val bluetoothAdapter: BluetoothAdapter? = bluetoothManager.adapter

        if (bluetoothAdapter == null) {
            this.finish()
        }

        if (bluetoothAdapter?.isEnabled == false) {
            val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            val startForResult = registerForActivityResult(
                ActivityResultContracts.StartActivityForResult()
            ) { result: ActivityResult ->
                if (result.resultCode == Activity.RESULT_OK) { this.finish() }
                recreate()
            }
            startForResult.launch(enableBtIntent)
        }

        val pairedDevices: Set<BluetoothDevice>? = bluetoothAdapter?.bondedDevices
        pairedDevices?.forEach { device ->
            val deviceName = device.name
            val deviceHardwareAddress = device.address
            Log.d("LOG_TAG", "Name: $deviceName | Addr: $deviceHardwareAddress")
        }

        if (pairedDevices?.isEmpty() == true) { this.finish() }
        var devices: List<BluetoothDevice> = pairedDevices!!.toList()

        setContent {
            SlonTheme {
                SlonApp(devices)
            }
        }
    }
}


@Composable
fun SlonApp(devicesList: List<BluetoothDevice>) {
    ConnectActivity(
        modifier = Modifier
            .fillMaxSize()
            .wrapContentSize(Alignment.Center),
        devicesList = devicesList
    )
}


@Composable
fun ConnectActivity(modifier: Modifier, devicesList: List<BluetoothDevice>) {
    LazyColumn(modifier = modifier) {
        items(devicesList.count()) { device ->
            DeviceCard(
                deviceName = devicesList[device].name,
                deviceAddr = devicesList[device].address,
                modifier = Modifier.padding(4.dp)
            )
        }
    }
}


@Composable
fun DeviceCard(deviceName: String, deviceAddr: String, modifier: Modifier = Modifier) {
    val context = LocalContext.current
    Card(modifier = modifier.fillMaxWidth().clickable {
        Log.d("LOG_TAG", "Name: $deviceName | Addr: $deviceAddr")
        val intent: Intent = Intent(context, GPSActivity::class.java)
        intent.putExtra("mac", deviceAddr)
        context.startActivity(intent)
    }) {
        Column {
            Text(
                text = deviceName,
                modifier = Modifier.padding(2.dp),
                style = MaterialTheme.typography.headlineSmall,
                fontSize = 20.sp
            )
            Text(
                text = deviceAddr,
                modifier = Modifier.padding(2.dp),
                style = MaterialTheme.typography.headlineSmall,
                fontSize = 20.sp
            )
        }
    }
}

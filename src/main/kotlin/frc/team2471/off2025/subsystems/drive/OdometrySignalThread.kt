// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.team2471.off2025.subsystems.drive

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.StatusSignal
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import frc.team2471.off2025.generated.TunerConstants
import frc.team2471.off2025.util.asMilliseconds
import frc.team2471.off2025.util.asSeconds
import frc.team2471.off2025.util.hertzToTime
import java.util.*
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.locks.ReentrantLock
import java.util.function.DoubleSupplier

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 *
 * This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
object OdometrySignalThread : Thread() {
    private val signalsLock = ReentrantLock() //Locks for thread-safety
    val odometryLock = ReentrantLock()

    private val isCANFD = CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD //CANFD protocol can run at higher frequency, are we using it?

    private var phoenixSignals: Array<BaseStatusSignal> = arrayOf() //List of ctre signals to record
    private val genericSignals: MutableList<DoubleSupplier> = ArrayList<DoubleSupplier>() //List of non-ctre signals to record

    private val phoenixQueues: MutableList<Queue<Double>> = ArrayList<Queue<Double>>() //List of queues that hold measurements for ctre devices (Kraken motors, Pigeon2 gyro)
    private val genericQueues: MutableList<Queue<Double>> = ArrayList<Queue<Double>>() //List of queues that hold measurements from non-ctre devices (navx gyro)
    private val timestampQueues: MutableList<Queue<Double>> = ArrayList<Queue<Double>>() //List of queues that contains the timestamps of the measurements

    const val ODOMETRY_FREQUENCY: Double = 100.0//if (isCANFD) 250.0 else 100.0

    init {
        setName("PhoenixOdometryThread")
        setDaemon(true)
    }

    override fun start() {
        if (timestampQueues.isNotEmpty()) {
            super.start()
        }
    }

    /** Registers a Phoenix signal to be read from the thread.  */
    fun registerSignal(signal: StatusSignal<Angle>): Queue<Double> {
        val queue: Queue<Double> = ArrayBlockingQueue<Double>(20)
        signalsLock.lock()
        odometryLock.lock()
        try {
            phoenixSignals = arrayOf(*phoenixSignals, signal)
            phoenixQueues.add(queue)
        } finally {
            signalsLock.unlock()
            odometryLock.unlock()
        }
        return queue
    }

    /** Registers a generic signal to be read from the thread.  */
    fun registerSignal(signal: DoubleSupplier): Queue<Double> {
        val queue: Queue<Double> = ArrayBlockingQueue<Double>(20)
        signalsLock.lock()
        odometryLock.lock()
        try {
            genericSignals.add(signal)
            genericQueues.add(queue)
        } finally {
            signalsLock.unlock()
            odometryLock.unlock()
        }
        return queue
    }

    /** Returns a new queue that returns timestamp values for each sample.  */
    fun makeTimestampQueue(): Queue<Double> {
        val queue: Queue<Double> = ArrayBlockingQueue(20)
        odometryLock.lock()
        try {
            timestampQueues.add(queue)
        } finally {
            odometryLock.unlock()
        }
        return queue
    }

    override fun run() {
        while (true) {
            // Wait for updates from all signals
            signalsLock.lock() //we do not want the phoenixSignals list to be modified from a separate thread while we are reading it from this thread
            try {
                if (phoenixSignals.isNotEmpty()) {
                    if (isCANFD) {
                        BaseStatusSignal.waitForAll(ODOMETRY_FREQUENCY.hertzToTime().asSeconds * 2.0, *phoenixSignals)
                    } else {
                        // "waitForAll" does not support blocking on multiple signals with a bus
                        // that is not CAN FD, regardless of Pro licensing. No reasoning for this
                        // behavior is provided by the documentation.
                        sleep(ODOMETRY_FREQUENCY.hertzToTime().asMilliseconds.toLong())
                        BaseStatusSignal.refreshAll(*phoenixSignals) // update all the CAN signals
                    }
                }
            } catch (e: InterruptedException) {
                e.printStackTrace()
            } finally {
                signalsLock.unlock()
            }

            // Save new data to queues
            odometryLock.lock()
            try {
                // Sample timestamp is current FPGA time minus average CAN latency
                //     Default timestamps from Phoenix are NOT compatible with
                //     FPGA timestamps, this solution is imperfect but close
                var timestamp = RobotController.getFPGATime() / 1e6
                var totalLatency = 0.0
                for (signal in phoenixSignals) {
                    totalLatency += signal.timestamp.latency
                }
                if (phoenixSignals.isNotEmpty()) {
                    timestamp -= totalLatency / phoenixSignals.size
                }

                // Add new samples to queues
                for (i in phoenixSignals.indices) {
                    phoenixQueues[i].offer(phoenixSignals[i].valueAsDouble)
                }
                for (i in genericSignals.indices) {
                    genericQueues[i].offer(genericSignals[i].asDouble)
                }
                for (i in timestampQueues.indices) {
                    timestampQueues[i].offer(timestamp)
                }
            } finally {
                odometryLock.unlock()
            }
        }
    }
}

/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.ctre.phoenix6.hardware;

import java.util.ArrayList;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;
import com.ctre.phoenix6.wpiutils.AutoFeedEnable;
import com.ctre.phoenix6.wpiutils.CallbackHelper;
import com.ctre.phoenix6.wpiutils.ReplayAutoEnable;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HAL.SimPeriodicBeforeCallback;
import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/**
 * WPILib-integrated version of {@link CoreCANrange}
 */
public class CANrange extends CoreCANrange implements Sendable, AutoCloseable {
    /**
     * The StatusSignal getters are copies so that calls
     * to the WPI interface do not update any references
     */
    private final StatusSignal<Distance> m_distanceGetter = getDistance(false).clone();
    private final StatusSignal<Boolean> m_isDetectedGetter = getIsDetected(false).clone();

    private static final DeviceType kSimDeviceType = DeviceType.P6_CANrangeType;

    private SimDevice m_simCANrange;
    private SimDouble m_simSupplyVoltage;
    private SimDouble m_simDistance;
    private SimBoolean m_simIsDetected;

    // returned registered callbacks
    private final ArrayList<CallbackStore> m_simValueChangedCallbacks = new ArrayList<CallbackStore>();
    private SimPeriodicBeforeCallback m_simPeriodicBeforeCallback = null;

    /**
     * Constructs a new CANrange object.
     * <p>
     * Constructs the device using the default CAN bus for the system:
     * <ul>
     *   <li>"rio" on roboRIO
     *   <li>"can0" on Linux
     *   <li>"*" on Windows
     * </ul>
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner.
     */
    public CANrange(int deviceId) {
        this(deviceId, "");
    }

    /**
     * Constructs a new CANrange object.
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner.
     * @param canbus   The CAN bus this device is on.
     */
    public CANrange(int deviceId, CANBus canbus) {
        this(deviceId, canbus.getName());
    }

    /**
     * Constructs a new CANrange object.
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner.
     * @param canbus   Name of the CAN bus this device is on. Possible CAN bus
     *                 strings are:
     *                 <ul>
     *                   <li>"rio" for the native roboRIO CAN bus
     *                   <li>CANivore name or serial number
     *                   <li>SocketCAN interface (non-FRC Linux only)
     *                   <li>"*" for any CANivore seen by the program
     *                   <li>empty string (default) to select the default for the
     *                       system:
     *                   <ul>
     *                     <li>"rio" on roboRIO
     *                     <li>"can0" on Linux
     *                     <li>"*" on Windows
     *                   </ul>
     *                 </ul>
     */
    public CANrange(int deviceId, String canbus) {
        super(deviceId, canbus);
        SendableRegistry.addLW(this, "CANrange (v6) ", deviceId);

        if (RobotBase.isSimulation()) {
            /* run in both swsim and hwsim */
            AutoFeedEnable.getInstance().start();
        }
        if (Utils.isReplay()) {
            ReplayAutoEnable.getInstance().start();
        }

        m_simCANrange = SimDevice.create("CAN:CANrange (v6)", deviceId);
        if (m_simCANrange != null) {
            m_simPeriodicBeforeCallback = HAL.registerSimPeriodicBeforeCallback(this::onPeriodic);

            m_simSupplyVoltage = m_simCANrange.createDouble("supplyVoltage", Direction.kInput, 12.0);
            m_simDistance = m_simCANrange.createDouble("distance", Direction.kInput, 0);

            m_simIsDetected = m_simCANrange.createBoolean("isDetected", Direction.kOutput, false);

            final SimDeviceSim sim = new SimDeviceSim("CAN:CANrange (v6)");
            m_simValueChangedCallbacks
                    .add(sim.registerValueChangedCallback(m_simSupplyVoltage, this::onValueChanged, true));
            m_simValueChangedCallbacks
                    .add(sim.registerValueChangedCallback(m_simDistance, this::onValueChanged, true));
        }
    }

    // ----- Auto-Closable ----- //
    @Override
    public void close() {
        SendableRegistry.remove(this);
        if (m_simPeriodicBeforeCallback != null) {
            m_simPeriodicBeforeCallback.close();
            m_simPeriodicBeforeCallback = null;
        }
        if (m_simCANrange != null) {
            m_simCANrange.close();
            m_simCANrange = null;
        }

        for (var callback : m_simValueChangedCallbacks) {
            callback.close();
        }
        m_simValueChangedCallbacks.clear();

        AutoFeedEnable.getInstance().stop();
    }

    // ----- Callbacks for Sim ----- //
    private void onValueChanged(String name, int handle, int direction, HALValue value) {
        String deviceName = SimDeviceDataJNI.getSimDeviceName(SimDeviceDataJNI.getSimValueDeviceHandle(handle));
        String physType = deviceName + ":" + name;
        PlatformJNI.JNI_SimSetPhysicsInput(
            kSimDeviceType.value, getDeviceID(),
            physType, CallbackHelper.getRawValue(value)
        );
    }

    private void onPeriodic() {
        double value = 0;
        int err = 0;

        final int deviceID = getDeviceID();

        value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "SupplyVoltage");
        err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
        if (err == 0) {
            m_simSupplyVoltage.set(value);
        }
        value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "Distance");
        err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
        if (err == 0) {
            m_simDistance.set(value);
        }
        value = PlatformJNI.JNI_SimGetPhysicsValue(kSimDeviceType.value, deviceID, "IsDetected");
        err = PlatformJNI.JNI_SimGetLastError(kSimDeviceType.value, deviceID);
        if (err == 0) {
            m_simIsDetected.set(value != 0.0);
        }
    }

    // ----- Sendable ----- //
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CANrange");
        builder.addDoubleProperty("Distance", () -> m_distanceGetter.refresh().getValueAsDouble(), null);
        builder.addBooleanProperty("Is Detected", () -> m_isDetectedGetter.refresh().getValue(), null);
    }
}

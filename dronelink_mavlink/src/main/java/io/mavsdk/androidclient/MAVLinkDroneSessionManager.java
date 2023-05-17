package io.mavsdk.androidclient;

import android.content.Context;
import android.util.Log;

import com.dronelink.core.DroneSession;
import com.dronelink.core.DroneSessionManager;
import com.dronelink.core.command.Command;
import com.dronelink.core.kernel.core.Message;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import io.mavsdk.MavsdkEventQueue;
import io.mavsdk.System;
import io.mavsdk.mavsdkserver.MavsdkServer;
import io.reactivex.disposables.Disposable;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class MAVLinkDroneSessionManager implements DroneSessionManager {

    private static final String TAG = MAVLinkDroneSessionManager.class.getCanonicalName();

    private final Context context;
    private MAVLinkDroneSession session;
    private final AtomicBoolean isRegistrationInProgress = new AtomicBoolean(false);
    private boolean registered = false;
    private final List<DroneSessionManager.Listener> listeners = new LinkedList<>();
    private System drone;
    public static final String BACKEND_IP_ADDRESS = "127.0.0.1";
    private MavsdkServer mavsdkServer = new MavsdkServer();
    private final List<Disposable> disposables = new ArrayList<>();
    private static final Logger logger = LoggerFactory.getLogger(MapsActivity.class);
    private boolean isMavsdkServerRunning = false;




    public MAVLinkDroneSessionManager(final Context context) {
        this.context = context;
    }


    @Override
    public void addListener(final DroneSessionManager.Listener listener) {
        listeners.add(listener);
        final DroneSession session = this.session;
        if (session != null) {
            listener.onOpened(session);
        }
    }

    @Override
    public void removeListener(final DroneSessionManager.Listener listener) {
        listeners.remove(listener);
    }

    @Override
    public void closeSession() {
        final DroneSession previousSession = session;
        if (previousSession != null) {
            previousSession.close();
            session = null;

            for (final DroneSessionManager.Listener listener : listeners) {
                listener.onClosed(previousSession);
            }
        }
    }

    @Override
    public void startRemoteControllerLinking(final Command.Finisher finisher) {
    }

    @Override
    public void stopRemoteControllerLinking(final Command.Finisher finisher) {
    }

    @Override
    public DroneSession getSession() {
        return session;
    }

    @Override
    public List<Message> getStatusMessages() {
        final List<Message> messages = new ArrayList<>();
        return messages;
    }

    public void register(final Context context) {
        if (registered) {
            return;
        }

        if (isRegistrationInProgress.compareAndSet(false, true)) {
            final MAVLinkDroneSessionManager self = this;

            MavsdkEventQueue.executor().execute(() -> {
                int mavsdkServerPort = mavsdkServer.run();
                drone = new System(BACKEND_IP_ADDRESS, mavsdkServerPort);

                Log.e(TAG, "STARTING MAVSDK SERVER" );

                disposables.add(drone.getTelemetry().getFlightMode().distinctUntilChanged()
                        .subscribe(flightMode -> logger.debug("flight mode: " + flightMode)));
                disposables.add(drone.getTelemetry().getArmed().distinctUntilChanged()
                        .subscribe(armed -> logger.debug("armed: " + armed)));
                disposables.add(drone.getTelemetry().getPosition().subscribe(position -> {
                }));

                isMavsdkServerRunning = true;
                registered = true;
                Log.e(TAG, "REGISTERED MAVSDK" );
            });
        }
    }
}

package io.mavsdk.androidclient;

import android.content.Context;
import android.graphics.Point;
import android.graphics.PointF;
import android.location.Location;
import android.os.Handler;
import android.util.Log;
import android.util.SparseArray;

import androidx.annotation.NonNull;

import com.dronelink.core.Convert;
import com.dronelink.core.DatedValue;
import com.dronelink.core.DroneControlSession;
import com.dronelink.core.DroneSession;
import com.dronelink.core.DroneSessionManager;
import com.dronelink.core.Dronelink;
import com.dronelink.core.Executor;
import com.dronelink.core.MissionExecutor;
import com.dronelink.core.ModeExecutor;
import com.dronelink.core.Version;
import com.dronelink.core.adapters.BatteryStateAdapter;
import com.dronelink.core.adapters.CameraStateAdapter;
import com.dronelink.core.adapters.DroneAdapter;
import com.dronelink.core.adapters.DroneStateAdapter;
import com.dronelink.core.adapters.GimbalStateAdapter;
import com.dronelink.core.adapters.RemoteControllerStateAdapter;
import com.dronelink.core.command.Command;
import com.dronelink.core.command.CommandConfig;
import com.dronelink.core.command.CommandError;
import com.dronelink.core.command.CommandQueue;
import com.dronelink.core.command.MultiChannelCommandQueue;
import com.dronelink.core.kernel.command.drone.CollisionAvoidanceDroneCommand;
import com.dronelink.core.kernel.command.drone.ConnectionFailSafeBehaviorDroneCommand;
import com.dronelink.core.kernel.command.drone.DroneCommand;
import com.dronelink.core.kernel.command.drone.FlightAssistantDroneCommand;
import com.dronelink.core.kernel.command.drone.HomeLocationDroneCommand;
import com.dronelink.core.kernel.command.drone.LandingProtectionDroneCommand;
import com.dronelink.core.kernel.command.drone.LowBatteryWarningThresholdDroneCommand;
import com.dronelink.core.kernel.command.drone.MaxAltitudeDroneCommand;
import com.dronelink.core.kernel.command.drone.MaxDistanceDroneCommand;
import com.dronelink.core.kernel.command.drone.MaxDistanceLimitationDroneCommand;
import com.dronelink.core.kernel.command.drone.PrecisionLandingDroneCommand;
import com.dronelink.core.kernel.command.drone.ReturnHomeAltitudeDroneCommand;
import com.dronelink.core.kernel.command.drone.ReturnHomeObstacleAvoidanceDroneCommand;
import com.dronelink.core.kernel.command.drone.ReturnHomeRemoteObstacleAvoidanceDroneCommand;
import com.dronelink.core.kernel.command.drone.SeriousLowBatteryWarningThresholdDroneCommand;
import com.dronelink.core.kernel.command.drone.SmartReturnHomeDroneCommand;
import com.dronelink.core.kernel.command.drone.UpwardsAvoidanceDroneCommand;
import com.dronelink.core.kernel.command.drone.VisionAssistedPositioningDroneCommand;
import com.dronelink.core.kernel.command.remotecontroller.RemoteControllerCommand;
import com.dronelink.core.kernel.command.remotecontroller.TargetGimbalChannelRemoteControllerCommand;
import com.dronelink.core.kernel.core.GeoCoordinate;
import com.dronelink.core.kernel.core.Message;
import com.dronelink.core.kernel.core.enums.ExecutionEngine;

import io.mavsdk.androidclient.adapters.MAVLinkDroneAdapter;
import io.mavsdk.androidclient.adapters.MAVLinkDroneStateAdapter;
import io.mavsdk.System;
import io.mavsdk.telemetry.Telemetry;

import org.json.JSONException;

import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MAVLinkDroneSession implements DroneSession {

    private static final String TAG = MAVLinkDroneSession.class.getCanonicalName();

    private final Context context;
    private final DroneSessionManager manager;
    private final MAVLinkDroneAdapter adapter;

    private final Date opened = new Date();
    private boolean closed = false;
    public boolean isClosed() {
        return closed;
    }

    private final List<Listener> listeners = new LinkedList<>();
    private final ExecutorService listenerExecutor = Executors.newSingleThreadExecutor();
    private final CommandQueue droneCommands = new CommandQueue();
    private final MultiChannelCommandQueue remoteControllerCommands = new MultiChannelCommandQueue();

    private final ExecutorService stateSerialQueue = Executors.newSingleThreadExecutor();
    private final MAVLinkDroneStateAdapter state;

    private final ExecutorService remoteControllerSerialQueue = Executors.newSingleThreadExecutor();
    private Date remoteControllerInitialized;
//    private DatedValue<HardwareState> remoteControllerState;

    public MAVLinkDroneSession(final Context context, final DroneSessionManager manager, final System drone) {
        this.context = context;
        this.state = new MAVLinkDroneStateAdapter(context, drone);
        this.manager = manager;
        this.adapter = new MAVLinkDroneAdapter(drone);
        initDrone();

        new Thread() {
            @Override
            public void run() {
                try {
                    while (!closed) {
                        if (!state.initialized && state.serialNumber != null && state.name != null && state.model != null && state.firmwarePackageVersion != null) {
                            state.initialized = true;
                            onInitialized();
                        }

                        final Location location = state.getLocation();
                        if (state.getLocation() != null) {
                            if (!state.located) {
                                state.located = true;
                                onLocated();
                            }

                            if (!state.isFlying()) {
                                state.lastKnownGroundLocation = location;
                            }
                        }

                        if (!state.initVirtualStickDisabled) {
//                            final FlightController flightController = drone.getFlightController();
                            final DatedValue<Telemetry> flightControllerState = state.flightControllerState;
                            if (drone != null && flightControllerState != null && flightControllerState.value != null) {
                                state.initVirtualStickDisabled = true;
                                if (flightControllerState.value.getFlightMode() != Telemetry.FlightMode.MISSION) {
                                    drone.getVirtualStickModeEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
                                        @Override
                                        public void onSuccess(final Boolean enabled) {
                                            if (enabled) {
                                                flightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                                                    @Override
                                                    public void onResult(final DJIError djiError) {
                                                        if (djiError == null) {
                                                            Log.i(TAG, "Flight controller virtual stick deactivated");
                                                        }
                                                    }
                                                });
                                            }
                                        }

                                        @Override
                                        public void onFailure(final DJIError djiError) {}
                                    });
                                }
                            }
                        }

                        if (remoteControllerInitialized == null) {
                            final RemoteController remoteController = adapter.getDrone().getRemoteController();
                            if (remoteController != null) {
                                initRemoteController(remoteController);
                            }
                        }

                        droneCommands.process();
                        remoteControllerCommands.process();

                        final MissionExecutor missionExecutor = Dronelink.getInstance().getMissionExecutor();
                        final ModeExecutor modeExecutor = Dronelink.getInstance().getModeExecutor();
                        final boolean missionExecutorEngaged = (missionExecutor != null && missionExecutor.isEngaged());
                    }

                    Log.i(TAG, "Drone session closed");
                }
                catch (final InterruptedException e) {}
            }
        }.start();
    }


    private void initDrone() {
        Log.i(TAG, "Drone session opened");

        final System drone = adapter.getDrone();
        drone.setDiagnosticsInformationCallback(new DJIDiagnostics.DiagnosticsInformationCallback() {
            @Override
            public void onUpdate(final List<DJIDiagnostics> list) {
                final List<Message> messages = new ArrayList<>();
                for (final DJIDiagnostics value : list) {
                    final Message message = DronelinkDJI.getMessage(value);
                    if (message != null) {
                        messages.add(message);
                    }
                }

                state.diagnosticsInformationMessages = new DatedValue<>(messages);
            }
        });

        if (drone.getFlightController() != null) {
            initFlightController(adapter.getDrone().getFlightController());
        }

        final RemoteController remoteController = drone.getRemoteController();
        if (remoteController != null) {
            initRemoteController(remoteController);
        }

        initListeners();
    }

    private void initFlightController(final FlightController flightController) {
        Log.i(TAG, "Flight controller connected");

        final System drone = adapter.getDrone();
        final Model model = drone.getModel();
        if (model == null) {
            state.model = "";
        }
        else {
            state.model = drone.getModel().getDisplayName();
            if (state.model == null) {
                state.model = "";
            } else {
                Log.i(TAG, "Model: " + state.model);
            }
        }

        state.firmwarePackageVersion = drone.getFirmwarePackageVersion();
        if (state.firmwarePackageVersion != null) {
            Log.i(TAG, "Firmware package version: " + state.firmwarePackageVersion);
        }

        drone.getName(new CommonCallbacks.CompletionCallbackWith<String>() {
            @Override
            public void onSuccess(final String s) {
                state.name = s;
                if (state.name == null) {
                    state.name = "";
                }
                else {
                    Log.i(TAG, "Name: " + s);
                }
            }

            @Override
            public void onFailure(final DJIError djiError) {

            }
        });

        initSerialNumber(flightController, 0);

        flightController.getMultipleFlightModeEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
            @Override
            public void onSuccess(final Boolean enabled) {
                if (!enabled) {
                    flightController.setMultipleFlightModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(final DJIError djiError) {
                            if (djiError == null) {
                                Log.i(TAG, "Flight controller multiple flight mode enabled");
                            }
                        }
                    });
                }
            }

            @Override
            public void onFailure(final DJIError djiError) {}
        });

        flightController.getNoviceModeEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
            @Override
            public void onSuccess(final Boolean enabled) {
                if (enabled) {
                    flightController.setNoviceModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(final DJIError djiError) {
                            if (djiError == null) {
                                Log.i(TAG, "Flight controller novice mode disabled");
                            }
                        }
                    });
                }
            }

            @Override
            public void onFailure(final DJIError djiError) {}
        });

        flightController.setStateCallback(new FlightControllerState.Callback() {
            private Double lastNonZeroFlyingAltitude = null;
            private boolean isFlyingPrevious = false;
            private boolean areMotorsOnPrevious = false;

            @Override
            public void onUpdate(@NonNull final FlightControllerState flightControllerStateUpdated) {
                stateSerialQueue.execute(new Runnable() {
                    @Override
                    public void run() {
                        if (isFlyingPrevious && !flightControllerStateUpdated.isFlying()) {
                            if (Dronelink.getInstance().droneOffsets.droneAltitudeContinuity) {
                                //automatically adjust the drone altitude offset if:
                                //1) altitude continuity is enabled
                                //2) the drone is going from flying to not flying
                                //3) the altitude reference is ground level
                                //4) the current drone altitude offset is not zero
                                //5) the last flight altitude is available
                                //6) the absolute value of last non-zero flying altitude is more than 1m
                                if ((Dronelink.getInstance().droneOffsets.droneAltitudeReference == null || Dronelink.getInstance().droneOffsets.droneAltitudeReference == 0) &&
                                        lastNonZeroFlyingAltitude != null && Math.abs(lastNonZeroFlyingAltitude) > 1) {
                                    //adjust by the last non-zero flying altitude
                                    Dronelink.getInstance().droneOffsets.droneAltitude -= lastNonZeroFlyingAltitude;
                                }
                            } else {
                                Dronelink.getInstance().droneOffsets.droneAltitude = 0;
                            }
                        }

                        state.flightControllerState = new DatedValue<>(flightControllerStateUpdated);
                        if (areMotorsOnPrevious != flightControllerStateUpdated.areMotorsOn()) {
                            onMotorsChanged(flightControllerStateUpdated.areMotorsOn());
                        }

                        isFlyingPrevious = flightControllerStateUpdated.isFlying();
                        areMotorsOnPrevious = flightControllerStateUpdated.areMotorsOn();

                        if (flightControllerStateUpdated.isFlying()) {
                            if (flightControllerStateUpdated.getAircraftLocation().getAltitude() != 0) {
                                lastNonZeroFlyingAltitude = (double)flightControllerStateUpdated.getAircraftLocation().getAltitude();
                            }
                        }
                        else {
                            lastNonZeroFlyingAltitude = null;
                        }
                    }
                });
            }
        });

        flightController.setASBInformationCallback(new AirSenseSystemInformation.Callback() {
            @Override
            public void onUpdate(@NonNull final AirSenseSystemInformation airSenseSystemInformation) {
                stateSerialQueue.execute(new Runnable() {
                    @Override
                    public void run() {
                        state.flightControllerAirSenseState = new DatedValue<>(airSenseSystemInformation);
                    }
                });
            }
        });

        final Compass compass = flightController.getCompass();
        if (compass != null) {
            compass.setCompassStateCallback(new CompassState.Callback() {
                @Override
                public void onUpdate(@NonNull final CompassState compassState) {
                    stateSerialQueue.execute(new Runnable() {
                        @Override
                        public void run() {
                            state.compassState = new DatedValue<>(compassState);
                        }
                    });
                }
            });
        }

        final Battery battery = drone.getBattery();
        if (battery != null) {
            battery.setStateCallback(new BatteryState.Callback() {
                @Override
                public void onUpdate(final BatteryState batteryState) {
                    stateSerialQueue.execute(new Runnable() {
                        @Override
                        public void run() {
                            state.batteryState = new DatedValue<>(batteryState);
                        }
                    });
                }
            });
        }

        final FlightAssistant flightAssistant = flightController.getFlightAssistant();
        if (flightAssistant != null) {
            flightAssistant.setVisionDetectionStateUpdatedCallback(new VisionDetectionState.Callback() {
                @Override
                public void onUpdate(@NonNull final VisionDetectionState visionDetectionState) {
                    if (visionDetectionState.getPosition() == VisionSensorPosition.NOSE) {
                        stateSerialQueue.execute(new Runnable() {
                            @Override
                            public void run() {
                                state.visionDetectionState = new DatedValue<>(visionDetectionState);
                            }
                        });
                    }
                }
            });
        }

        final AirLink airlink = adapter.getDrone().getAirLink();
        if (airlink != null) {
            airlink.setUplinkSignalQualityCallback(new SignalQualityCallback() {
                @Override
                public void onUpdate(int i) {
                    state.uplinkSignalQuality = new DatedValue<>(i);
                }
            });

            airlink.setDownlinkSignalQualityCallback(new SignalQualityCallback() {
                @Override
                public void onUpdate(int i) {
                    state.downlinkSignalQuality = new DatedValue<>(i);
                }
            });
        }
    }

    private void initRemoteController(final RemoteController remoteController) {
        remoteControllerInitialized = new Date();
        remoteController.setHardwareStateCallback(new HardwareState.HardwareStateCallback() {
            @Override
            public void onUpdate(@NonNull final HardwareState hardwareState) {
                remoteControllerSerialQueue.execute(new Runnable() {
                    @Override
                    public void run() {
                        remoteControllerState = new DatedValue<>(hardwareState);
                    }
                });
            }
        });
    }

    private void initListeners() {
        startListeningForChanges(FlightControllerKey.create(FlightControllerKey.MAX_FLIGHT_HEIGHT), (oldValue, newValue) -> stateSerialQueue.execute(() -> {
            if (newValue instanceof Integer)
                state.maxFlightHeight = new DatedValue<>((Integer) newValue);
            else if (oldValue instanceof Integer)
                state.maxFlightHeight = new DatedValue<>((Integer) oldValue);
            else
                state.maxFlightHeight = null;
        }));

        startListeningForChanges(FlightControllerKey.create(FlightControllerKey.LOW_BATTERY_WARNING_THRESHOLD), (oldValue, newValue) -> stateSerialQueue.execute(() -> {
            if (newValue instanceof Integer)
                state.lowBatteryWarningThreshold = new DatedValue<>((Integer) newValue);
            else if (oldValue instanceof Integer)
                state.lowBatteryWarningThreshold = new DatedValue<>((Integer) oldValue);
            else
                state.lowBatteryWarningThreshold = null;
        }));

        startListeningForChanges(RemoteControllerKey.create(RemoteControllerKey.CONTROLLING_GIMBAL_INDEX), (oldValue, newValue) -> {
            if (newValue instanceof Integer) {
                state.remoteControllerGimbalChannel = new DatedValue<>((Integer) newValue);
            }
            else {
                state.remoteControllerGimbalChannel = null;
            }
        });
    }

    private void startListeningForChanges(final DJIKey key, final KeyListener listener) {
        djiKeyListeners.add(listener);
        final KeyManager manager = DJISDKManager.getInstance().getKeyManager();
        if (manager == null) {
            return;
        }

        manager.addListener(key, listener);
        manager.getValue(key, new GetCallback() {
            @Override
            public void onSuccess(@NonNull final Object newValue) {
                listener.onValueChange(null, newValue);
            }

            @Override
            public void onFailure(@NonNull final DJIError djiError) {
                Log.e("KeyedListener", "Error retrieving key: " + key);
            }
        });
    }

    public void componentConnected(final BaseComponent component) {
        if (component instanceof FlightController) {
            initFlightController((FlightController)component);
        }
        else if (component instanceof Camera) {
            initCamera((Camera)component);
        }
        else if (component instanceof Gimbal) {
            initGimbal((Gimbal)component);
        }
        else if (component instanceof RemoteController) {
            initRemoteController((RemoteController)component);
        }
    }

    public void componentDisconnected(final System component) {
        if (component != null) {
            Log.i(TAG, "Flight controller disconnected");
            stateSerialQueue.execute(new Runnable() {
                @Override
                public void run() {
                    state.flightControllerState = null;
//                    state.visionDetectionState = null;
                }
            });
        }
    }

    public MAVLinkDroneAdapter getAdapter() {
        return adapter;
    }

    @Override
    public DroneSessionManager getManager() {
        return manager;
    }

    @Override
    public DroneAdapter getDrone() {
        return adapter;
    }

    @Override
    public DatedValue<DroneStateAdapter> getState() {
        try {
            return stateSerialQueue.submit(new Callable<DatedValue<DroneStateAdapter>>() {
                @Override
                public DatedValue<DroneStateAdapter> call() {
                    return state.toDatedValue();
                }
            }).get();
        }
        catch (final ExecutionException | InterruptedException e) {
            return null;
        }
    }

    public DatedValue<FlightControllerState> getFlightControllerState() {
        try {
            return stateSerialQueue.submit(new Callable<DatedValue<FlightControllerState>>() {
                @Override
                public DatedValue<FlightControllerState> call() {
                    return state.flightControllerState;
                }
            }).get();
        }
        catch (final ExecutionException | InterruptedException e) {
            return null;
        }
    }

    @Override
    public Date getOpened() {
        return opened;
    }

    @Override
    public String getId() {
        return state.id;
    }

    @Override
    public String getManufacturer() {
        return "DJI";
    }

    @Override
    public String getSerialNumber() {
        return null;
    }

    @Override
    public String getName() {
        return state.name;
    }

    @Override
    public String getModel() {
        return state.model;
    }

    @Override
    public String getFirmwarePackageVersion() {
        return state.firmwarePackageVersion;
    }

    @Override
    public boolean isInitialized() {
        return state.initialized;
    }

    @Override
    public boolean isLocated() {
        return state.located;
    }

    @Override
    public boolean isTelemetryDelayed() {
        return System.currentTimeMillis() - getState().date.getTime() > 2000;
    }

    @Override
    public Message getDisengageReason() {
        if (closed) {
            return new Message(context.getString(R.string.MissionDisengageReason_drone_disconnected_title));
        }

        if (adapter.getDrone().getFlightController() == null) {
            return new Message(context.getString(R.string.MissionDisengageReason_drone_control_unavailable_title));
        }

        final DatedValue<FlightControllerState> flightControllerState = state.flightControllerState;
        if (flightControllerState == null || flightControllerState.value == null) {
            return new Message(context.getString(R.string.MissionDisengageReason_telemetry_unavailable_title));
        }

        if (isTelemetryDelayed()) {
            return new Message(context.getString(R.string.MissionDisengageReason_telemetry_delayed_title), context.getString(R.string.MissionDisengageReason_telemetry_delayed_details));
        }

        if (flightControllerState.value.hasReachedMaxFlightHeight()) {
            return new Message(context.getString(R.string.MissionDisengageReason_drone_max_altitude_title), context.getString(R.string.MissionDisengageReason_drone_max_altitude_details));
        }

        if (flightControllerState.value.hasReachedMaxFlightRadius()) {
            return new Message(context.getString(R.string.MissionDisengageReason_drone_max_distance_title), context.getString(R.string.MissionDisengageReason_drone_max_distance_details));
        }

        return null;
    }

    @Override
    public void identify(final String id) {
        state.id = id;
    }

    @Override
    public void addListener(final Listener listener) {
        final DroneSession self = this;
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                listeners.add(listener);

                if (state.initialized) {
                    listener.onInitialized(self);
                }

                if (state.located) {
                    listener.onLocated(self);
                }
            }
        });
    }

    @Override
    public void removeListener(final Listener listener) {
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                listeners.remove(listener);
            }
        });
    }

    private void onInitialized() {
        final MAVLinkDroneSession self = this;
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                for (final Listener listener : listeners) {
                    listener.onInitialized(self);
                }
            }
        });
    }

    private void onLocated() {
        final MAVLinkDroneSession self = this;
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                for (final Listener listener : listeners) {
                    listener.onLocated(self);
                }
            }
        });
    }

    private void onMotorsChanged(final boolean value) {
        final MAVLinkDroneSession self = this;
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                for (final Listener listener : listeners) {
                    listener.onMotorsChanged(self, value);
                }
            }
        });
    }

    private void onCommandExecuted(final com.dronelink.core.kernel.command.Command command) {
        final MAVLinkDroneSession self = this;
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                for (final Listener listener : listeners) {
                    listener.onCommandExecuted(self, command);
                }
            }
        });
    }

    private void onCommandFinished(final com.dronelink.core.kernel.command.Command command, final CommandError error) {
        final MAVLinkDroneSession self = this;
        listenerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                for (final Listener listener : listeners) {
                    listener.onCommandFinished(self, command, error);
                }
            }
        });
    }


    @Override
    public void addCommand(final com.dronelink.core.kernel.command.Command command) throws Dronelink.UnregisteredException, CommandTypeUnhandledException {
        Command.Executor executor = null;

        if (command instanceof DroneCommand) {
            executor = new Command.Executor() {
                @Override
                public CommandError execute(final Command.Finisher finished) {
                    onCommandExecuted(command);
                    return executeDroneCommand((DroneCommand)command, finished);
                }
            };
        }
        else if (command instanceof RemoteControllerCommand) {
            executor = new Command.Executor() {
                @Override
                public CommandError execute(final Command.Finisher finished) {
                    onCommandExecuted(command);
                    return executeRemoteControllerCommand((RemoteControllerCommand) command, finished);
                }
            };
        }

        if (executor != null) {
            final Command c = new Command(
                    command,
                    executor,
                    new Command.Finisher() {
                        @Override
                        public void execute(final CommandError error) {
                            onCommandFinished(command, error);
                        }
                    },
                    command.getConfig());

            if (c.config.retriesEnabled == null) {
                //disable retries when the DJI SDK reports that the product does not support the feature
                c.config.retriesEnabled = new CommandConfig.RetriesEnabled() {
                    @Override
                    public boolean execute(final CommandError error) {
                        if (error != null && error.code == DJIError.COMMAND_NOT_SUPPORTED_BY_HARDWARE.getErrorCode()) {
                            return false;
                        }
                        return true;
                    }
                };
            }

            if (command instanceof DroneCommand) {
                droneCommands.addCommand(c);
            }
            else if (command instanceof RemoteControllerCommand) {
                remoteControllerCommands.addCommand(((RemoteControllerCommand)command).channel, c);
            }
            return;
        }

        throw new CommandTypeUnhandledException();
    }

    @Override
    public void removeCommands() {
        droneCommands.removeAll();
        remoteControllerCommands.removeAll();
    }

    @Override
    public DroneControlSession createControlSession(final Context context, final ExecutionEngine executionEngine, final Executor executor) throws UnsupportedExecutionEngineException, UnsupportedDroneDJIExecutionEngineException {
        switch (executionEngine) {
            case DRONELINK_KERNEL:
                return new MAVLinkVirtualStickSession(context, this);

            case DJI:
                switch (adapter.drone.getModel()) {
                    case MAVIC_MINI:
                    case DJI_MINI_2:
                    case DJI_MINI_SE:
                    case MAVIC_AIR_2:
                    case DJI_AIR_2S:
                    case MATRICE_300_RTK:
                        throw new UnsupportedDroneDJIExecutionEngineException();

                    default:
                        break;
                }

                if (executor instanceof MissionExecutor) {
                    try {
                        return new DJIWaypointMissionSession(context, this, (MissionExecutor)executor);
                    } catch (final JSONException e) {
                        throw new UnsupportedExecutionEngineException(executionEngine);
                    }
                }
                break;

            default:
                break;
        }

        throw new UnsupportedExecutionEngineException(executionEngine);
    }

    @Override
    public DatedValue<RemoteControllerStateAdapter> getRemoteControllerState(final int channel) {
        try {
            return remoteControllerSerialQueue.submit(new Callable<DatedValue<RemoteControllerStateAdapter>>() {
                @Override
                public DatedValue<RemoteControllerStateAdapter> call() {
                    if (remoteControllerState == null) {
                        return null;
                    }

                    final RemoteControllerStateAdapter remoteControllerStateAdapter = new DJIRemoteControllerStateAdapter(remoteControllerState.value);
                    return new DatedValue<>(remoteControllerStateAdapter, remoteControllerState.date);
                }
            }).get();
        }
        catch (final ExecutionException | InterruptedException e) {
            return null;
        }
    }

    @Override
    public DatedValue<CameraStateAdapter> getCameraState(int i) {
        return null;
    }

    @Override
    public DatedValue<GimbalStateAdapter> getGimbalState(int i) {
        return null;
    }


    @Override
    public DatedValue<BatteryStateAdapter> getBatteryState(final int index) {
        //TODO
        return null;
    }

    @Override
    public void resetPayloads() {

    }

    @Override
    public void resetPayloads(boolean b, boolean b1) {

    }


    @Override
    public void close() {
        this.closed = true;
    }

    protected void sendResetVelocityCommand(final CommonCallbacks.CompletionCallback completion) {
        adapter.sendResetVelocityCommand(completion);
    }


    private CommonCallbacks.CompletionCallback createCompletionCallback(final Command.Finisher finished) {
        return new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(final io.mavsdk.System mavlinkError) {
                finished.execute(DronelinkMAVLink.createCommandError(mavlinkError));
            }
        };
    }

    private <V> CommonCallbacks.CompletionCallbackWith<V> createCompletionCallbackWith(final Command.FinisherWith<V> success, final Command.Finisher error) {
        return new CommonCallbacks.CompletionCallbackWith<V>() {
            @Override
            public void onSuccess(final V value) {
                success.execute(value);
            }

            @Override
            public void onFailure(final io.mavsdk.System mavlinkError) {
                error.execute((DronelinkMAVLink.createCommandError(mavlinkError)));
            }
        };
    }

    private CommandError executeDroneCommand(final DroneCommand command, final Command.Finisher finished) {
        if (command instanceof FlightAssistantDroneCommand) {
            return executeFlightAssistantDroneCommand((FlightAssistantDroneCommand) command, finished);
        }

        final FlightController flightController = adapter.getDrone().getFlightController();
        if (flightController == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_control_unavailable_title));
        }

        if (command instanceof ConnectionFailSafeBehaviorDroneCommand) {
            flightController.getConnectionFailSafeBehavior(createCompletionCallbackWith(new Command.FinisherWith<ConnectionFailSafeBehavior>() {
                @Override
                public void execute(final ConnectionFailSafeBehavior current) {
                    final ConnectionFailSafeBehavior target = DronelinkDJI.getDroneConnectionFailSafeBehavior(((ConnectionFailSafeBehaviorDroneCommand) command).connectionFailSafeBehavior);
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightController.setConnectionFailSafeBehavior(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof HomeLocationDroneCommand) {
            final GeoCoordinate coordinate = ((HomeLocationDroneCommand) command).coordinate;
            flightController.setHomeLocation(DronelinkDJI.getCoordinate(coordinate), createCompletionCallback(finished));
            return null;
        }

        if (command instanceof LowBatteryWarningThresholdDroneCommand) {
            flightController.getLowBatteryWarningThreshold(createCompletionCallbackWith(new Command.FinisherWith<Integer>() {
                @Override
                public void execute(final Integer current) {
                    final Integer target = (int)(((LowBatteryWarningThresholdDroneCommand) command).lowBatteryWarningThreshold * 100);
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightController.setLowBatteryWarningThreshold(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof MaxAltitudeDroneCommand) {
            flightController.getMaxFlightHeight(createCompletionCallbackWith(new Command.FinisherWith<Integer>() {
                @Override
                public void execute(final Integer current) {
                    final Integer target = (int)(((MaxAltitudeDroneCommand) command).maxAltitude);
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightController.setMaxFlightHeight(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof MaxDistanceDroneCommand) {
            flightController.getMaxFlightRadius(createCompletionCallbackWith(new Command.FinisherWith<Integer>() {
                @Override
                public void execute(final Integer current) {
                    final Integer target = (int)(((MaxDistanceDroneCommand) command).maxDistance);
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightController.setMaxFlightRadius(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof MaxDistanceLimitationDroneCommand) {
            flightController.getMaxFlightRadiusLimitationEnabled(createCompletionCallbackWith(new Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((MaxDistanceLimitationDroneCommand) command).enabled;
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightController.setMaxFlightRadiusLimitationEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof ReturnHomeAltitudeDroneCommand) {
            flightController.getGoHomeHeightInMeters(createCompletionCallbackWith(new Command.FinisherWith<Integer>() {
                @Override
                public void execute(final Integer current) {
                    final Integer target = (int)(((ReturnHomeAltitudeDroneCommand) command).returnHomeAltitude);
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightController.setGoHomeHeightInMeters(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof SeriousLowBatteryWarningThresholdDroneCommand) {
            flightController.getSeriousLowBatteryWarningThreshold(createCompletionCallbackWith(new Command.FinisherWith<Integer>() {
                @Override
                public void execute(final Integer current) {
                    final Integer target = (int)(((SeriousLowBatteryWarningThresholdDroneCommand) command).seriousLowBatteryWarningThreshold * 100);
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightController.setSeriousLowBatteryWarningThreshold(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof SmartReturnHomeDroneCommand) {
            flightController.getSmartReturnToHomeEnabled(createCompletionCallbackWith(new Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((SmartReturnHomeDroneCommand) command).enabled;
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightController.setSmartReturnToHomeEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        return new CommandError(context.getString(R.string.MissionDisengageReason_command_type_unhandled));
    }

    private CommandError executeFlightAssistantDroneCommand(final FlightAssistantDroneCommand command, final Command.Finisher finished) {
        final FlightController flightController = adapter.getDrone().getFlightController();
        if (flightController == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_flight_assistant_unavailable_title));
        }

        final FlightAssistant flightAssistant = flightController.getFlightAssistant();
        if (flightAssistant == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_flight_assistant_unavailable_title));
        }

        if (command instanceof CollisionAvoidanceDroneCommand) {
//            flightAssistant.getCollisionAvoidanceEnabled(createCompletionCallbackWith(new Command.FinisherWith<Boolean>() {
//                @Override
//                public void execute(final Boolean current) {
//                    final Boolean target = ((CollisionAvoidanceDroneCommand) command).enabled;
//                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
//                        @Override
//                        public void execute() {
//                            flightAssistant.setCollisionAvoidanceEnabled(target, createCompletionCallback(finished));
//                        }
//                    });
//                }
//            }, finished));
            //skipping conditional execution for now because it seems like the DJI SDK always returns true for getCollisionAvoidanceEnabled
            flightAssistant.setCollisionAvoidanceEnabled(((CollisionAvoidanceDroneCommand) command).enabled, createCompletionCallback(finished));
            return null;
        }

        if (command instanceof LandingProtectionDroneCommand) {
            flightAssistant.getLandingProtectionEnabled(createCompletionCallbackWith(new Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((LandingProtectionDroneCommand) command).enabled;
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightAssistant.setLandingProtectionEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof PrecisionLandingDroneCommand) {
            flightAssistant.getPrecisionLandingEnabled(createCompletionCallbackWith(new Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((PrecisionLandingDroneCommand) command).enabled;
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightAssistant.setPrecisionLandingEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof ReturnHomeObstacleAvoidanceDroneCommand) {
            flightAssistant.getRTHObstacleAvoidanceEnabled(createCompletionCallbackWith(new Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((ReturnHomeObstacleAvoidanceDroneCommand) command).enabled;
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightAssistant.setRTHObstacleAvoidanceEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof ReturnHomeRemoteObstacleAvoidanceDroneCommand) {
            flightAssistant.getRTHRemoteObstacleAvoidanceEnabled(createCompletionCallbackWith(new Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((ReturnHomeRemoteObstacleAvoidanceDroneCommand) command).enabled;
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightAssistant.setRTHRemoteObstacleAvoidanceEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof UpwardsAvoidanceDroneCommand) {
            flightAssistant.getUpwardVisionObstacleAvoidanceEnabled(createCompletionCallbackWith(new Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((UpwardsAvoidanceDroneCommand) command).enabled;
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightAssistant.setUpwardVisionObstacleAvoidanceEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        if (command instanceof VisionAssistedPositioningDroneCommand) {
            flightAssistant.getVisionAssistedPositioningEnabled(createCompletionCallbackWith(new Command.FinisherWith<Boolean>() {
                @Override
                public void execute(final Boolean current) {
                    final Boolean target = ((VisionAssistedPositioningDroneCommand) command).enabled;
                    Command.conditionallyExecute(!target.equals(current), finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            flightAssistant.setVisionAssistedPositioningEnabled(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        return new CommandError(context.getString(R.string.MissionDisengageReason_command_type_unhandled));
    }

    private CommandError executeRemoteControllerCommand(final RemoteControllerCommand command, final Command.Finisher finished) {
        final RemoteController remoteController = DronelinkDJI.getRemoteController(adapter.getDrone(), command.channel);
        if (remoteController == null) {
            return new CommandError(context.getString(R.string.MissionDisengageReason_drone_remote_controller_unavailable_title));
        }

        if (command instanceof TargetGimbalChannelRemoteControllerCommand) {
            remoteController.getControllingGimbalIndex(createCompletionCallbackWith(new Command.FinisherWith<Integer>() {
                @Override
                public void execute(final Integer current) {
                    final int target = ((TargetGimbalChannelRemoteControllerCommand) command).targetGimbalChannel;
                    Command.conditionallyExecute(target != current, finished, new Command.ConditionalExecutor() {
                        @Override
                        public void execute() {
                            remoteController.setControllingGimbalIndex(target, createCompletionCallback(finished));
                        }
                    });
                }
            }, finished));
            return null;
        }

        return new CommandError(context.getString(R.string.MissionDisengageReason_command_type_unhandled));
    }


}

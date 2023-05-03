package io.mavsdk.androidclient.adapters;

import com.dronelink.core.adapters.RemoteControllerAdapter;

import io.mavsdk.manual_control.ManualControl;

public class MAVLinkRemoteControllerAdapter  implements RemoteControllerAdapter {
    public final ManualControl remoteController;

    public MAVLinkRemoteControllerAdapter(final ManualControl remoteController) {
        this.remoteController = remoteController;
    }

    @Override
    public int getIndex() {
        return 0;
    }
}

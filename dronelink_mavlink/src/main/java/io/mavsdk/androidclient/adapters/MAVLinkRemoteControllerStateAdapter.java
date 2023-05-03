package io.mavsdk.androidclient.adapters;

import com.dronelink.core.adapters.RemoteControllerStateAdapter;
import com.dronelink.core.kernel.core.RemoteControllerButton;
import com.dronelink.core.kernel.core.RemoteControllerStick;
import com.dronelink.core.kernel.core.RemoteControllerWheel;

import io.mavsdk.manual_control.ManualControl;

public class MAVLinkRemoteControllerStateAdapter implements RemoteControllerStateAdapter {
    public final ManualControl state;

//    public MAVLinkRemoteControllerStateAdapter(final HardwareState state) {
//        this.state = state;
//    }
//
//    public RemoteControllerStick getLeftStick() {
//        return state == null || state.getLeftStick() == null ? null : new RemoteControllerStick(state.getLeftStick().getHorizontalPosition() / 660.0, state.getLeftStick().getVerticalPosition() / 660.0);
//    }

//    public RemoteControllerWheel getLeftWheel() {
//        return state == null ? null : new RemoteControllerWheel(true, false, (double)state.getLeftDial() / 660.0);
//    }

//    public RemoteControllerStick getRightStick() {
//        return state == null || state.getRightStick() == null ? null : new RemoteControllerStick(state.getRightStick().getHorizontalPosition() / 660.0, state.getRightStick().getVerticalPosition() / 660.0);
//    }

//    public RemoteControllerButton getPauseButton() {
//        return state == null || state.getPauseButton() == null ? null : new RemoteControllerButton(state.getPauseButton().isPresent(), state.getPauseButton().isClicked());
//    }
//
//    public RemoteControllerButton getReturnHomeButton() {
//        return state == null || state.getGoHomeButton() == null ? null : new RemoteControllerButton(state.getGoHomeButton().isPresent(), state.getGoHomeButton().isClicked());
//    }
//
//    public RemoteControllerButton getC1Button() {
//        return state == null || state.getC1Button() == null ? null : new RemoteControllerButton(state.getC1Button().isPresent(), state.getC1Button().isClicked());
//    }
//
//    public RemoteControllerButton getC2Button() {
//        return state == null || state.getC2Button() == null ? null : new RemoteControllerButton(state.getC2Button().isPresent(), state.getC2Button().isClicked());
//    }
}

package org.firstinspires.ftc.teamcode.Enabled_Classes.rr.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

public class LynxOptimizedI2cFactory {
    private static class BetterI2cDeviceSynchImplOnSimple extends I2cDeviceSynchImplOnSimple {
        private BetterI2cDeviceSynchImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
            super(simple, isSimpleOwned);
        }

        @Override
        public void setReadWindow(ReadWindow window) {
            // intentionally do nothing
        }
    }

    private LynxOptimizedI2cFactory() {

    }

    /**
     * Constructs an "optimized" REV {@link I2cDeviceSynch} for use with an SDK I2C device
     * driver.
     * @param module corresponding lynx module
     * @param bus bus this device is attached to (these are confusingly labeled ports on the module)
     * @return "optimized" I2cDeviceSynch
     */
    public static I2cDeviceSynch createLynxI2cDeviceSynch(LynxModule module, int bus) {
        return new BetterI2cDeviceSynchImplOnSimple(
                LynxFirmwareVersionManager.createLynxI2cDeviceSynch(AppUtil.getDefContext(), module, bus
                ), true);
    }

    /**
     * Constructs an "optimized" LynxEmbeddedIMU.
     *
     * @see #createLynxI2cDeviceSynch(LynxModule, int)
     */
    public static LynxEmbeddedIMU createLynxEmbeddedImu(LynxModule module, int bus) {
        return new LynxEmbeddedIMU(createLynxI2cDeviceSynch(module, bus));
    }
}

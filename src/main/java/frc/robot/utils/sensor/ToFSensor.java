package frc.robot.utils.sensor;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.CAN;

import java.nio.ByteBuffer;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millisecond;

public class ToFSensor {
    private final int id;
    private final CAN can;
    private final CANData data;
    private final ScheduledExecutorService scheduler;
    private double sensorValue;

    /**
     * Creates the sensor.
     *
     * @param id CAN ID
     */
    public ToFSensor(int id) {
        this.id = id;
        this.can = new CAN(id);
        this.data = new CANData();
        this.scheduler = Executors.newSingleThreadScheduledExecutor();
        scheduler.scheduleAtFixedRate(this::update, 20, 20, TimeUnit.MILLISECONDS);
    }

    /**
     * Updates the sensor data.
     */
    private void update() {
        if (receiveLatest()) {
            parseByte(data.data);
        }
    }

    /**
     * Updates the {@link CANData}
     *
     * @return if it was a success
     */
    private boolean receiveLatest() {
        return can.readPacketLatest(id, data);
    }

    /**
     * Uses a {@link ByteBuffer} to read the byte array into sensorValue.
     */
    private void parseByte(byte[] bytes) {
        sensorValue = ByteBuffer.wrap(bytes).getDouble();
    }

    /**
     * Set's the period for reading from the can bus. Default: 20 ms
     * The Roborio can bus runs at a period of 20 ms.
     *
     * @param period scheduler period
     */
    private void setUpdatePeriod(Time period) {
        scheduler.shutdownNow();
        long newPeriod = (long) period.in(Millisecond);
        scheduler.scheduleAtFixedRate(this::update, newPeriod, newPeriod, TimeUnit.MILLISECONDS);
    }

    /**
     * Get the latest packet as a double.
     */
    public Distance getDistance() {
        return Meters.of(sensorValue);
    }
}

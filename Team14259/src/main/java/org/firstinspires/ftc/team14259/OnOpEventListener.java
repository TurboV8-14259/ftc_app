package org.firstinspires.ftc.team14259;

interface OnOpEventListener {
    // this can be any type of method
    boolean onOpIsActiveEvent();

    // Add Telemetry Data
    void onTelemetryAddData(String sDataItem, String sData);

    // Add Telemetry Data
    void onTelemetryUpdate();
}

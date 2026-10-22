# Forest-Sensing-Device-Code
Arduino-based forest monitoring system integrating Edge Impulse acoustic classification, environmental sensors, and hybrid solar–hydrogen fuel-cell power management for potential illegal logging detection.

/*
 * FOREST SURVEILLANCE NODE
 * Arduino Nano 33 BLE Sense Rev2
 *
 * Features:
 * - Continuous operation without deep sleep
 * - Edge Impulse chainsaw sound classification
 * - Temperature, humidity and pressure sensing
 * - Bluetooth Low Energy telemetry
 * - Automatic BLE advertising after disconnect
 * - Microphone timeout and diagnostic reporting
 *
 * IMPORTANT:
 * Install the Edge Impulse Arduino library exported
 * from your trained project.
 */

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_HS300x.h>
#include <Arduino_LPS22HB.h>
#include <PDM.h>
#include <Chainsaw_inferencing.h>
#include <math.h>
#include <string.h>

// ============================================
// CONFIGURATION
// ============================================

#define AUDIO_SAMPLE_RATE EI_CLASSIFIER_FREQUENCY

#define AUDIO_BUFFER_SIZE 512

#define AUDIO_TIMEOUT_MS 10000

#define SENSOR_INTERVAL_MS 3000

#define AI_INTERVAL_MS 3000

#define CHAINSAW_THRESHOLD 0.70f

// ============================================
// AUDIO BUFFERS
// ============================================

// Full audio window required by Edge Impulse
static int16_t inferenceBuffer[
    EI_CLASSIFIER_RAW_SAMPLE_COUNT
];

// Two temporary audio buffers
static int16_t audioBuffers[2][AUDIO_BUFFER_SIZE];

static volatile uint8_t readyBuffer = 0;
static volatile bool bufferReady = false;
static volatile bool bufferOverflow = false;
static volatile bool recording = false;

static volatile size_t audioBufferIndex = 0;
static volatile uint8_t writeBuffer = 0;

// ============================================
// BLE CONFIGURATION
// ============================================

BLEService nodeService(
    "19B10000-E8F2-537E-4F6C-D104768A1214"
);

BLEStringCharacteristic tempChar(
    "19B10001-E8F2-537E-4F6C-D104768A1214",
    BLERead | BLENotify,
    20
);

BLEStringCharacteristic humChar(
    "19B10002-E8F2-537E-4F6C-D104768A1214",
    BLERead | BLENotify,
    20
);

BLEStringCharacteristic pressChar(
    "19B10003-E8F2-537E-4F6C-D104768A1214",
    BLERead | BLENotify,
    20
);

BLEStringCharacteristic aiChar(
    "19B10004-E8F2-537E-4F6C-D104768A1214",
    BLERead | BLENotify,
    100
);

// ============================================
// SYSTEM STATE
// ============================================

unsigned long lastSensorTime = 0;
unsigned long lastAITime = 0;

bool temperatureReady = false;
bool pressureReady = false;

bool bleConnectedPreviously = false;

// ============================================
// PDM MICROPHONE CALLBACK
// ============================================

void onPDMdata()
{
    int bytesAvailable = PDM.available();

    while (bytesAvailable > 0) {

        // If the previous buffer has not been
        // consumed, discard incoming samples.
        if (bufferReady) {
            bufferOverflow = true;

            int16_t discard[64];

            while (PDM.available() > 0) {
                int n = PDM.available();
                if (n > (int)sizeof(discard)) {
                    n = sizeof(discard);
                }

                if (PDM.read(
                    (char*)discard, n
                ) <= 0) {
                    break;
                }
            }

            return;
        }

        size_t remaining =
            AUDIO_BUFFER_SIZE - audioBufferIndex;

        int bytesToRead =
            (int)(remaining * sizeof(int16_t));

        if (bytesAvailable < bytesToRead) {
            bytesToRead = bytesAvailable;
        }

        int bytesRead = PDM.read(
            (char*)&audioBuffers[writeBuffer]
                               [audioBufferIndex],
            bytesToRead
        );

        if (bytesRead <= 0) {
            return;
        }

        audioBufferIndex +=
            bytesRead / sizeof(int16_t);

        if (audioBufferIndex >= AUDIO_BUFFER_SIZE) {

            readyBuffer = writeBuffer;
            bufferReady = true;

            writeBuffer = 1 - writeBuffer;
            audioBufferIndex = 0;
        }

        bytesAvailable = PDM.available();
    }
}

// ============================================
// EDGE IMPULSE SIGNAL CALLBACK
// ============================================

int audioSignalGetData(
    size_t offset,
    size_t length,
    float *outPtr
)
{
    if (offset + length >
        EI_CLASSIFIER_RAW_SAMPLE_COUNT) {
        return -1;
    }

    numpy::int16_to_float(
        &inferenceBuffer[offset],
        outPtr,
        length
    );

    return 0;
}

// ============================================
// RECORD AUDIO
// ============================================

bool recordAudio()
{
    Serial.println("[AUDIO] Starting capture...");

    size_t samplesCaptured = 0;

    noInterrupts();

    bufferReady = false;
    bufferOverflow = false;
    audioBufferIndex = 0;
    writeBuffer = 0;
    recording = true;

    interrupts();

    unsigned long startTime = millis();

    while (samplesCaptured <
           EI_CLASSIFIER_RAW_SAMPLE_COUNT) {

        BLE.poll();

        if (millis() - startTime >
            AUDIO_TIMEOUT_MS) {

            recording = false;

            Serial.println(
                "[ERROR] Audio capture timeout!"
            );

            return false;
        }

        if (bufferOverflow) {

            recording = false;

            Serial.println(
                "[ERROR] Audio buffer overflow!"
            );

            return false;
        }

        if (bufferReady) {

            uint8_t index;
            size_t copyCount;

            noInterrupts();

            index = readyBuffer;

            size_t remaining =
                EI_CLASSIFIER_RAW_SAMPLE_COUNT
                - samplesCaptured;

            copyCount = remaining;

            if (copyCount > AUDIO_BUFFER_SIZE) {
                copyCount = AUDIO_BUFFER_SIZE;
            }

            // Copy while the callback cannot
            // modify the completed buffer.
            memcpy(
                &inferenceBuffer[samplesCaptured],
                audioBuffers[index],
                copyCount * sizeof(int16_t)
            );

            bufferReady = false;

            interrupts();

            samplesCaptured += copyCount;
        }
    }

    recording = false;

    Serial.print("[AUDIO] Captured samples: ");
    Serial.println(samplesCaptured);

    return true;
}

// ============================================
// ENVIRONMENTAL SENSORS
// ============================================

void updateEnvironmentalSensors()
{
    float temperature = NAN;
    float humidity = NAN;
    float pressure = NAN;

    if (temperatureReady) {
        temperature = HS300x.readTemperature();
        humidity = HS300x.readHumidity();
    }

    if (pressureReady) {
        pressure = BARO.readPressure();
    }

    Serial.println();
    Serial.println("===== ENVIRONMENT =====");

    Serial.print("Temperature: ");
    Serial.println(temperature);

    Serial.print("Humidity: ");
    Serial.println(humidity);

    Serial.print("Pressure: ");
    Serial.println(pressure);

    if (BLE.connected()) {

        if (!isnan(temperature)) {
            tempChar.writeValue(
                String(temperature, 1) + " C"
            );
        }

        if (!isnan(humidity)) {
            humChar.writeValue(
                String(humidity, 1) + " %"
            );
        }

        if (!isnan(pressure)) {
            pressChar.writeValue(
                String(pressure, 1) + " kPa"
            );
        }
    }
}

// ============================================
// EDGE IMPULSE INFERENCE
// ============================================

void runAudioInference()
{
    Serial.println();
    Serial.println("==========================");
    Serial.println("EDGE IMPULSE INFERENCE");
    Serial.println("==========================");

    if (!recordAudio()) {

        Serial.println(
            "[AI] Recording failed."
        );

        return;
    }

    signal_t signal;

    signal.total_length =
        EI_CLASSIFIER_RAW_SAMPLE_COUNT;

    signal.get_data =
        &audioSignalGetData;

    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR error =
        run_classifier(
            &signal,
            &result,
            false
        );

    if (error != EI_IMPULSE_OK) {

        Serial.print(
            "[ERROR] Classifier error: "
        );

        Serial.println((int)error);

        return;
    }

    float highestConfidence = -1.0f;

    const char *topLabel = "Unknown";

    Serial.println();
    Serial.println("Classification results:");

    for (size_t i = 0;
         i < EI_CLASSIFIER_LABEL_COUNT;
         i++) {

        float confidence =
            result.classification[i].value;

        const char *label =
            result.classification[i].label;

        Serial.print(label);
        Serial.print(": ");
        Serial.println(confidence, 4);

        if (confidence > highestConfidence) {

            highestConfidence = confidence;
            topLabel = label;
        }
    }

    Serial.println();
    Serial.print("Top prediction: ");
    Serial.println(topLabel);

    Serial.print("Confidence: ");
    Serial.println(highestConfidence, 4);

    Serial.print("DSP time (ms): ");
    Serial.println(result.timing.dsp);

    Serial.print("Classification time (ms): ");
    Serial.println(
        result.timing.classification
    );

    // BLE telemetry
    String aiMessage =
        String(topLabel) +
        ":" +
        String(highestConfidence, 2);

    aiChar.writeValue(aiMessage);

    // Detection based on top prediction
    // NOTE: Adapt the label to the exact
    // class name in your Edge Impulse model.

    String prediction =
        String(topLabel);

    prediction.toLowerCase();

    if (prediction.indexOf("chainsaw") >= 0 &&
        highestConfidence >= CHAINSAW_THRESHOLD) {

        Serial.println();
        Serial.println(
            "***** CHAINSAW DETECTED *****"
        );

        aiChar.writeValue(
            "CHAINSAW ALERT"
        );
    }
}

// ============================================
// BLE INITIALIZATION
// ============================================

void initializeBLE()
{
    if (!BLE.begin()) {

        Serial.println(
            "[ERROR] BLE initialization failed!"
        );

        while (true) {
            delay(1000);
        }
    }

    BLE.setLocalName("ForestNode");

    BLE.setAdvertisedService(nodeService);

    nodeService.addCharacteristic(tempChar);
    nodeService.addCharacteristic(humChar);
    nodeService.addCharacteristic(pressChar);
    nodeService.addCharacteristic(aiChar);

    BLE.addService(nodeService);

    tempChar.writeValue("0.0 C");
    humChar.writeValue("0.0 %");
    pressChar.writeValue("0.0 kPa");
    aiChar.writeValue("Starting");

    BLE.advertise();

    Serial.println(
        "[BLE] Advertising as ForestNode"
    );
}

// ============================================
// SETUP
// ============================================

void setup()
{
    Serial.begin(115200);

    unsigned long serialStart = millis();

    while (!Serial &&
           millis() - serialStart < 3000) {
        delay(10);
    }

    Serial.println();
    Serial.println(
        "FOREST SURVEILLANCE NODE V2"
    );

    Serial.println(
        "Arduino Nano 33 BLE Sense Rev2"
    );

    Serial.print("Model frequency: ");
    Serial.println(AUDIO_SAMPLE_RATE);

    Serial.print("Required audio samples: ");
    Serial.println(
        EI_CLASSIFIER_RAW_SAMPLE_COUNT
    );

    delay(500);

    // Environmental sensors

    temperatureReady = HS300x.begin();

    if (temperatureReady) {
        Serial.println("[OK] HS300x sensor");
    } else {
        Serial.println("[FAIL] HS300x sensor");
    }

    pressureReady = BARO.begin();

    if (pressureReady) {
        Serial.println("[OK] Pressure sensor");
    } else {
        Serial.println("[FAIL] Pressure sensor");
    }

    // Microphone

    PDM.onReceive(onPDMdata);

    PDM.setBufferSize(1024);

    if (!PDM.begin(1, AUDIO_SAMPLE_RATE)) {

        Serial.println(
            "[FATAL] PDM microphone failed!"
        );

        while (true) {
            delay(1000);
        }
    }

    Serial.println("[OK] PDM microphone");

    // Bluetooth

    initializeBLE();

    Serial.println();
    Serial.println(
        "SYSTEM INITIALIZED"
    );

    Serial.println(
        "Continuous surveillance active."
    );
}

// ============================================
// MAIN LOOP
// ============================================

void loop()
{
    BLE.poll();

    BLEDevice central = BLE.central();

    bool isConnected =
        central && central.connected();

    if (isConnected &&
        !bleConnectedPreviously) {

        Serial.println(
            "[BLE] Device connected."
        );
    }

    if (!isConnected &&
        bleConnectedPreviously) {

        Serial.println(
            "[BLE] Device disconnected."
        );

        BLE.advertise();
    }

    bleConnectedPreviously = isConnected;

    unsigned long currentTime = millis();

    // Environmental sensor updates

    if (currentTime - lastSensorTime >=
        SENSOR_INTERVAL_MS) {

        lastSensorTime = currentTime;

        updateEnvironmentalSensors();
    }

    // Edge Impulse inference

    if (currentTime - lastAITime >=
        AI_INTERVAL_MS) {

        lastAITime = currentTime;

        runAudioInference();
    }
}

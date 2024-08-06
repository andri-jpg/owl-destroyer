#define BLYNK_TEMPLATE_ID "**********"
#define BLYNK_TEMPLATE_NAME "owldtc"
#include <andri12-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <BlynkSimpleEsp32.h>
#include <base64.h>
#include "esp_camera.h"
#include <NewPing.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WebServer.h>

// Inisialisasi WebServer pada port 80
WebServer server(80);

#define TRIGGER_PIN 13 // Pin untuk trigger ultrasonik
#define ECHO_PIN 12    // Pin untuk echo ultrasonik
#define MAX_DISTANCE 200 // Maksimal jarak untuk pengukuran (dalam cm)

// Inisialisasi sensor ultrasonik
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define LED_PIN           4 
#define BUZZER_PIN        14
#define STUN              15

char auth[] = "********";
char ssid[] = "***";
char pass[] = "*******k";

#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture

BlynkTimer timer; // Deklarasi BlynkTimer
BlynkTimer streamingTimer; // Timer untuk streaming
int streamingTimerID = -1; // ID untuk streaming timer

// Variabel untuk melacak mode saat ini
bool isStreamingMode = false;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);

/**
* @brief      Arduino setup function
*/
void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); 

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off initially
    pinMode(STUN, OUTPUT);
    digitalWrite(STUN, HIGH);

    Serial.begin(115200);
    while (!Serial);
    Blynk.begin(auth, ssid, pass);
    Serial.println("Edge Impulse Inferencing Demo");

    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else {
        ei_printf("Camera initialized\r\n");
        digitalWrite(LED_PIN, LOW); 
    }

    if (!SPIFFS.begin(true)) {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    timer.setInterval(500L, sendUltrasonicDistance); // Kirim data jarak setiap 1 detik

    setupWiFi();
    startCameraServer();
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    Blynk.run();
    timer.run();
    streamingTimer.run();
    server.handleClient();
    delay(1);

    if (!isStreamingMode) {
        handle_inferencing();
    }
}

void handle_inferencing() {
    if (isStreamingMode) return;

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);

    #if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) continue;
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                  bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }
    #else
    ei_printf("Predictions:\r\n");
    float owl_prediction = 0;
    float nothing_prediction = 1;

    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);

        if (strcmp(ei_classifier_inferencing_categories[i], "owl") == 0) {
            owl_prediction = result.classification[i].value;
        }
        if (strcmp(ei_classifier_inferencing_categories[i], "nothing") == 0) {
            nothing_prediction = result.classification[i].value;
        }
    }
    String prediction = "Owl: " + String(owl_prediction) + ", Nothing: " + String(nothing_prediction);
    Blynk.virtualWrite(V2, prediction);

    if (owl_prediction > 0.5 && nothing_prediction < 0.6) {
        for (int i = 0; i < 3; i++) {
            digitalWrite(BUZZER_PIN, HIGH); 
            delay(100); 
            digitalWrite(BUZZER_PIN, LOW); 
            delay(100); 
        }
        digitalWrite(STUN, LOW);
        delay(1000);
        digitalWrite(STUN, HIGH);
    } else {
        digitalWrite(STUN, HIGH);
    }
    #endif

    #if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
    #endif

    free(snapshot_buf);
}

BLYNK_WRITE(V1) {
    int pinValue = param.asInt();
    if (pinValue == 1) {
        handle_mode_switch(true);
    } else {
        handle_mode_switch(false);
    }
}

BLYNK_WRITE(V4) {
    int pinValue = param.asInt();
    if (pinValue == 1) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
}

void sendUltrasonicDistance() {
    unsigned int uS = sonar.ping();
    float distanceCM = uS / US_ROUNDTRIP_CM; 

    Serial.print("Distance: ");
    Serial.print(distanceCM);
    Serial.println(" cm");

    Blynk.virtualWrite(V3, distanceCM);
}

bool ei_camera_init(void) {
    if (is_initialised) return true;

    #if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
    #endif

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }

    #if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    #elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
    #endif

    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        ei_printf("Camera deinit failed\n");
        return;
    }
    is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

    esp_camera_fb_return(fb);

    if(!converted){
        ei_printf("Conversion failed\n");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    return 0;
}

void handle_mode_switch(bool streamingMode) {
    isStreamingMode = streamingMode;
    if (isStreamingMode) {
        digitalWrite(LED_PIN, HIGH); 
        streamingTimerID = streamingTimer.setInterval(33L, handle_jpg_stream); // Jalankan setiap 33ms (~30fps)
    } else {
        digitalWrite(LED_PIN, LOW);
        if (streamingTimerID != -1) {
            streamingTimer.deleteTimer(streamingTimerID); // Hentikan timer streaming
            streamingTimerID = -1;
        }
    }
}

void handle_jpg_stream() {
    WiFiClient client = server.client();
    camera_fb_t * fb = NULL;
    uint8_t * _jpg_buf = NULL;
    size_t _jpg_buf_len = 0;

    static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
    static const char* _STREAM_BOUNDARY = "\r\n--frame\r\n";
    static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

    client.printf("HTTP/1.1 200 OK\r\n");
    client.printf("Content-Type: %s\r\n\r\n", _STREAM_CONTENT_TYPE);

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return;
    }
    if(fb->format != PIXFORMAT_JPEG){
        bool converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        if(!converted){
            Serial.println("JPEG compression failed");
            esp_camera_fb_return(fb);
            return;
        }
    } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
    }

    client.printf(_STREAM_BOUNDARY);
    client.printf(_STREAM_PART, _jpg_buf_len);
    client.write(_jpg_buf, _jpg_buf_len);

    if(fb->format != PIXFORMAT_JPEG){
        free(_jpg_buf);
    }
    esp_camera_fb_return(fb);
    if(!client.connected()){
        handle_mode_switch(false); // Matikan mode streaming jika klien terputus
    }
}

void startCameraServer() {
    server.on("/", HTTP_GET, handle_jpg_stream);  // Direct root to video stream
    server.on("/switch_mode", HTTP_GET, []() {
        handle_mode_switch(!isStreamingMode);
        server.send(200, "text/plain", "Mode switched");
    });

    server.begin();  // Memulai server
    Serial.println("Web server started");
}

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");

    String ipAddress = WiFi.localIP().toString();
    Blynk.virtualWrite(V6, ipAddress);
    Serial.println("IP Address: " + ipAddress); 
}

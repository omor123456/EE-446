#include "TensorFlowLite.h"
#include "autoencoder_model.cc"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"

#include <Arduino_BMI270_BMM150.h>
#include <math.h>

tflite::MicroErrorReporter micro_error_reporter;
tflite::ErrorReporter* error_reporter = &micro_error_reporter;

// ---------------------------------------------------------------------------
// These constants must match the notebook exactly. See the "DEPLOYMENT
// CONSTANTS" cell of TinyML_Lab6_Student_TODO.ipynb.
// ---------------------------------------------------------------------------
const int kWindowSize = 100;        // WINDOW_SIZE in the notebook
const int kInputSize = 300;         // 100 time steps x 3 axes, index = t * 3 + axis
const int kSamplePeriodMs = 20;     // mHealth was recorded at 50 Hz -> 20 ms per sample

// The model was trained on mHealth data, which is stored in m/s^2.
// IMU.readAcceleration() on the BMI270 returns g, so every sample must be
// converted before it is quantized. Without this the input is ~9.8x too small
// and every window looks like near-zero motion.
const float kGravity = 9.80665f;

const int kTensorArenaSize = 64 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

// Accelerometer buffer (m/s^2)
float window_buffer[kWindowSize][3];
int sample_index = 0;
bool window_ready = false;
unsigned long last_sample_ms = 0;

// TFLite variables
tflite::MicroInterpreter* interpreter;
TfLiteTensor* input;
TfLiteTensor* output;

// Quantization parameters (read from the model at runtime)
float input_scale;
int input_zero_point;
float output_scale;
int output_zero_point;

// Threshold selected in the notebook (TODO 4).
// Normal windows peaked at ~1.66 MSE and the quietest anomaly was ~2.56, so 2.0
// sits near the middle of the empty band between the two distributions.
const float kReconstructionErrorThreshold = 2.0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1);
  }

  Serial.println("IMU initialized.");
  Serial.print("Accelerometer sample rate (Hz): ");
  Serial.println(IMU.accelerationSampleRate());

  const tflite::Model* model = tflite::GetModel(g_model);

  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema version mismatch.");
    while (1);
  }

  static tflite::AllOpsResolver resolver;

  static tflite::MicroInterpreter static_interpreter(
    model,
    resolver,
    tensor_arena,
    kTensorArenaSize,
    error_reporter
  );

  interpreter = &static_interpreter;

  Serial.println("Allocating tensors.");

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors failed.");
    while (1);
  }

  Serial.println("AllocateTensors successful.");

  input = interpreter->input(0);
  output = interpreter->output(0);

  input_scale = input->params.scale;
  input_zero_point = input->params.zero_point;
  output_scale = output->params.scale;
  output_zero_point = output->params.zero_point;

  Serial.print("Input scale / zero point: ");
  Serial.print(input_scale, 8);
  Serial.print(" / ");
  Serial.println(input_zero_point);

  Serial.print("Output scale / zero point: ");
  Serial.print(output_scale, 8);
  Serial.print(" / ");
  Serial.println(output_zero_point);

  Serial.print("Anomaly threshold (MSE): ");
  Serial.println(kReconstructionErrorThreshold, 4);

  Serial.println("Model setup complete.");
  last_sample_ms = millis();
}

void loop() {
  float x, y, z;

  // Sample at a fixed 50 Hz so the 100-sample window covers the same 2 seconds
  // of motion the model was trained on.
  if (millis() - last_sample_ms >= (unsigned long)kSamplePeriodMs) {
    if (IMU.accelerationAvailable()) {
      last_sample_ms += kSamplePeriodMs;

      IMU.readAcceleration(x, y, z);  // returns g

      window_buffer[sample_index][0] = x * kGravity;  // convert to m/s^2
      window_buffer[sample_index][1] = y * kGravity;
      window_buffer[sample_index][2] = z * kGravity;

      sample_index++;

      // A full window of 100 consecutive, in-order samples is ready.
      if (sample_index >= kWindowSize) {
        sample_index = 0;
        window_ready = true;
      }
    }
  }

  if (window_ready) {
    window_ready = false;  // consume this window; the next one starts filling now

    // Quantize the window into the int8 input tensor.
    // Flatten order must match the notebook: index = timestep * 3 + axis.
    for (int i = 0; i < kWindowSize; i++) {
      for (int j = 0; j < 3; j++) {
        float val = window_buffer[i][j];
        int index = i * 3 + j;

        int32_t q = lround(val / input_scale) + input_zero_point;
        q = constrain(q, -128, 127);

        input->data.int8[index] = static_cast<int8_t>(q);
      }
    }

    if (interpreter->Invoke() != kTfLiteOk) {
      Serial.println("Inference failed.");
      return;
    }

    // Reconstruction error = MSE between the original window and the
    // dequantized reconstruction, in the same units the notebook used.
    float recon_error = 0.0;

    for (int i = 0; i < kInputSize; i++) {
      float original = window_buffer[i / 3][i % 3];

      int8_t quant_pred = output->data.int8[i];
      float predicted = (quant_pred - output_zero_point) * output_scale;

      float diff = original - predicted;
      recon_error += diff * diff;
    }

    recon_error /= kInputSize;

    Serial.print("Reconstruction error: ");
    Serial.println(recon_error, 6);

    if (recon_error > kReconstructionErrorThreshold) {
      Serial.println("Result: Anomaly detected");
    } else {
      Serial.println("Result: Normal activity");
    }
  }
}

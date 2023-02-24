#include "pch.h"
#include "AirAPI_Win.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "endian.h"
#include "hidapi-win/include/hidapi.h"
#include "cglm/cglm.h"
#include <Windows.h>
#include <iostream>
#include <mutex>

//Air USB VID and PID
#define AIR_VID 0x3318
#define AIR_PID 0x0424

//Is Tracking
bool g_isTracking = true;

// ticks are in nanoseconds, 1000 Hz packets
#define TICK_LEN (1.0f / 1E9f)

// based on 24bit signed int w/ FSR = +/-2000 dps, datasheet option
#define GYRO_SCALAR (1.0f / 8388608.0f * 2000.0f)

// based on 24bit signed int w/ FSR = +/-16 g, datasheet option
#define ACCEL_SCALAR (1.0f / 8388608.0f * 16.0f)

static int rows, cols;
static versor rotation = GLM_QUAT_IDENTITY_INIT;
static vec3 ang_vel = {}, accel_vec = {};
static  FusionEuler euler;
static  FusionVector earth;
static FusionQuaternion qt;

hid_device* device;



#define SAMPLE_RATE (1000) // replace this with actual sample rate

std::mutex mtx;

typedef struct {
	uint64_t tick;
	int32_t ang_vel[3];
	int32_t accel[3];
} air_sample;

static int
parse_report(const unsigned char* buffer, int size, air_sample* out_sample)
{
	if (size != 64) {
		printf("Invalid packet size");
		return -1;
	}
	// clock in nanoseconds
	buffer += 4;
	out_sample->tick = ((uint64_t) * (buffer++)) | (((uint64_t) * (buffer++)) << 8) | (((uint64_t) * (buffer++)) << 16) | (((uint64_t) * (buffer++)) << 24)
		| (((uint64_t) * (buffer++)) << 32) | (((uint64_t) * (buffer++)) << 40) | (((uint64_t) * (buffer++)) << 48) | (((uint64_t) * (buffer++)) << 56);

	// gyroscope measurements
	buffer += 6;
	if (*(buffer + 2) & 0x80) {
		out_sample->ang_vel[0] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}
	else {
		out_sample->ang_vel[0] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	if (*(buffer + 2) & 0x80) {
		out_sample->ang_vel[1] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}
	else {
		out_sample->ang_vel[1] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	if (*(buffer + 2) & 0x80) {
		out_sample->ang_vel[2] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}
	else {
		out_sample->ang_vel[2] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	// accelerometer data
	buffer += 6;
	if (*(buffer + 2) & 0x80) {
		out_sample->accel[0] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}
	else {
		out_sample->accel[0] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	if (*(buffer + 2) & 0x80) {
		out_sample->accel[1] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}
	else {
		out_sample->accel[1] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	if (*(buffer + 2) & 0x80) {
		out_sample->accel[2] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}
	else {
		out_sample->accel[2] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	return 0;
}

static void
process_ang_vel(const int32_t in_ang_vel[3], vec3 out_vec)
{
	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_ang_vel[0]) * -1.0f * GYRO_SCALAR;
	out_vec[1] = (float)(in_ang_vel[2]) * GYRO_SCALAR;
	out_vec[2] = (float)(in_ang_vel[1]) * GYRO_SCALAR;
}

static void
process_accel(const int32_t in_accel[3], vec3 out_vec)
{
	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_accel[0]) * ACCEL_SCALAR;
	out_vec[1] = (float)(in_accel[2]) * ACCEL_SCALAR;
	out_vec[2] = (float)(in_accel[1]) * ACCEL_SCALAR;
}

static void
update_rotation(float dt, vec3 in_ang_vel)
{
	float ang_vel_length = glm_vec3_norm(in_ang_vel);

	if (ang_vel_length > 0.0001f) {
		vec3 rot_axis = { in_ang_vel[0] / ang_vel_length, in_ang_vel[1] / ang_vel_length, in_ang_vel[2] / ang_vel_length };
		float rot_angle = ang_vel_length * dt;

		versor delta_rotation;
		glm_quatv(delta_rotation, rot_angle, rot_axis);
		glm_quat_mul(rotation, delta_rotation, rotation);
	}

	glm_quat_normalize(rotation);
}


static hid_device*
open_device()
{
	struct hid_device_info* devs = hid_enumerate(AIR_VID, AIR_PID);
	struct hid_device_info* cur_dev = devs;
	hid_device* device = NULL;

	while (devs) {
		if (cur_dev->interface_number == 3) {
			device = hid_open_path(cur_dev->path);
			break;
		}

		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);
	return device;
}

struct ThreadParams {
	hid_device* device;
};

DWORD WINAPI track(LPVOID lpParam) {
	printf("THREAD BEGIN\n");
	//Thread to handle tracking
	unsigned char buffer[64] = {};
	uint64_t last_sample_tick = 0;
	air_sample sample = {};
	ThreadParams* params = static_cast<ThreadParams*>(lpParam);

	// Define calibration (replace with actual calibration data if available)
	const FusionMatrix gyroscopeMisalignment = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
	const FusionVector gyroscopeSensitivity = { 1.0f, 1.0f, 1.0f };
	const FusionVector gyroscopeOffset = { 0.0f, 0.0f, 0.0f };
	const FusionMatrix accelerometerMisalignment = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
	const FusionVector accelerometerSensitivity = { 1.0f, 1.0f, 1.0f };
	const FusionVector accelerometerOffset = { 0.0f, 0.0f, 0.0f };
	const FusionMatrix softIronMatrix = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
	const FusionVector hardIronOffset = { 0.0f, 0.0f, 0.0f };

	// Initialise algorithms
	FusionOffset offset;
	FusionAhrs ahrs;
	
	FusionOffsetInitialise(&offset, SAMPLE_RATE);
	FusionAhrsInitialise(&ahrs);

	// Set AHRS algorithm settings
	const FusionAhrsSettings settings = {
			.gain = 0.5f,
			.accelerationRejection = 10.0f,
			.magneticRejection = 20.0f,
			.rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
	};
	FusionAhrsSetSettings(&ahrs, &settings);
		printf("THREAD: While begin\n");
	while (g_isTracking) {

		//try read
		//int res = hid_read(device, buffer, sizeof(buffer));

		try {
			// code that might throw an exception
			int res = hid_read(device, buffer, sizeof(buffer));
			if (res < 0) {
				printf("Unable to get feature report\n");
				//break;
			}
		}
		catch (const std::exception& e) {
			// handle the exception
			printf("THREAD: ERR\n");
		std::cerr << e.what();
		}



		

		//parse
		parse_report(buffer, sizeof(buffer), &sample);

		//process sample
		process_ang_vel(sample.ang_vel, ang_vel);
		process_accel(sample.accel, accel_vec);

		// Acquire latest sensor data
		const uint64_t timestamp = sample.tick; // replace this with actual gyroscope timestamp
		FusionVector gyroscope = { ang_vel[0], ang_vel[1], ang_vel[2] }; // replace this with actual gyroscope data in degrees/s
		FusionVector accelerometer = { accel_vec[0], accel_vec[1], accel_vec[2] }; // replace this with actual accelerometer data in g

		// Apply calibration
		gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
		accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

		// Update gyroscope offset correction algorithm
		gyroscope = FusionOffsetUpdate(&offset, gyroscope);

		// Calculate delta time (in seconds) to account for gyroscope sample clock error
		static uint64_t previousTimestamp;
		const float deltaTime = (float)(timestamp - previousTimestamp) / (float)1e9;
		previousTimestamp = timestamp;

		// Update gyroscope AHRS algorithm
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);


		// Print algorithm outputs
		mtx.lock();
		
		qt = FusionAhrsGetQuaternion(&ahrs);
		euler = FusionQuaternionToEuler(qt);
		earth = FusionAhrsGetEarthAcceleration(&ahrs);
		update_rotation(deltaTime, ang_vel);
		mtx.unlock();
		

		

		//printf_s("Euler Roll: %f", euler.angle.roll);
	}
	return 0;
}



int StartConnection()
{
	printf("Opening Device\n");
	// open device
	device = open_device();
	if (!device) {
		printf("Unable to open device\n");
		return 1;
	}

	printf("Sending Payload\n");
	// open the floodgates
	uint8_t magic_payload[] = { 0x00, 0xaa, 0xc5, 0xd1, 0x21, 0x42, 0x04, 0x00, 0x19, 0x01 };
	int res = hid_write(device, magic_payload, sizeof(magic_payload));
	if (res < 0) {
		printf("Unable to write to device\n");
		return 1;
	}
	ThreadParams params = { device };
	g_isTracking = true;
	printf("Starting Thread\n");
	//Start Tracking Thread
	HANDLE trackThread = CreateThread(NULL, 0, track, &params, 0, NULL);
	if (trackThread == NULL) {
		std::cout << "Failed to create thread" << std::endl;
		return 1;
	}
	printf("Thread Started\n");

    return 1;
}


int StopConnection()
{
	g_isTracking = false;
	return 1;
}

float* GetQuaternion()
{
	mtx.lock();
	float* q = new float[4];
	q[0] = qt.array[0];
	q[1] = qt.array[1];
	q[2] = qt.array[2];
	q[3] = qt.array[3];
	mtx.unlock();
	return q;
}

float* GetEuler()
{
	mtx.lock();
	float* e = new float[3];
	
	e[0] = euler.angle.pitch;
	e[1] = euler.angle.roll;
	e[2] = euler.angle.yaw;
	mtx.unlock();
	return e;
}
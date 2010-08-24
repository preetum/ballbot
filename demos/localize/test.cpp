#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

#define min(x,y) ((x)<(y) ? x : y)
#define max(x,y) ((x)>(y) ? x : y)
#define TILT_MAX  38
#define TILT_MIN  -29
#define PAN_MAX   60
#define PAN_MIN   -60

class Camera {
	public:
	Camera() {
		pan = 0;
		tilt = 0;
	}

	/* Reset pan and tilt angle to 0 */
	void initialize() {
		system("/usr/bin/uvcdynctrl -s 'Pan Reset' 1");
		usleep(3000000L);
		system("/usr/bin/uvcdynctrl -s 'Tilt Reset' 1");
		usleep(1000000L);
	}

	/* Set pan angle in degrees */
	void setPan(int8_t value) {
		// make sure value is in range
		value = min(value, PAN_MAX);
		value = max(value, PAN_MIN);

		// calculate relative pan value (signs are intentionally reversed)
		int32_t delta = (int)(pan - value) * 64;

		// store new pan value
		pan = value;

		char cmd[60];
		sprintf(cmd, "/usr/bin/uvcdynctrl -s 'Pan (relative)' -- %d", delta);
		system(cmd);
	}

	/* Set tilt angle in degrees */
	void setTilt(int8_t value) {
		// make sure value is in range
		value = min(value, TILT_MAX);
		value = max(value, TILT_MIN);

		// calculate relative tilt value (signs are intentionally reversed)
		int32_t delta = (int)(tilt - value) * 64;

		// store new tilt value
		tilt = value;

		char cmd[60];
		sprintf(cmd, "/usr/bin/uvcdynctrl -s 'Tilt (relative)' -- %d", delta);
		system(cmd);
	}

	private:
	int8_t pan, tilt;
};

int main (void) {
	Camera c;
	c.initialize();
	printf("init\n");
	usleep(2000000);

	c.setPan(50);
	usleep(1000000);
	c.setPan(-50);
	usleep(1000000);
	c.setPan(0);
	usleep(1000000);
	c.setPan(100);
	usleep(1000000);
	c.setPan(-100);
	usleep(1000000);
	c.setPan(0);
	c.setTilt(15);
	usleep(1000000);
	c.setTilt(-15);
	usleep(1000000);
	c.setTilt(0);
	usleep(1000000);
	c.setTilt(40);
	usleep(1000000);
	c.setTilt(-40);
	usleep(1000000);
	c.setTilt(0);
}

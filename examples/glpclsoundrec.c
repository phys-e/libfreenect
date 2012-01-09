/* rMary_glpcsoundrec.c
 * Combines libfreenect's 'glpcview' and 'wavrec' to simultaneously record wav data
 * (4 channels) and gl point-cloud image from kinect
 *
 * Eric A. Dieckman (WM)
 * 05 Jan 2012
 * Last edited: 05 January 2012 EAD
*/

 /*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * Andrew Miller <amiller@dappervision.com>
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

// glpcview part:

#include "libfreenect.h"
#include "libfreenect_sync.h"
#include "libfreenect-audio.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include <stdlib.h>
#include <math.h>

// AUDIO PART:
static freenect_context* f_ctx;
static freenect_device* f_dev;
int die = 0;

char wavheader[] = {
	0x52, 0x49, 0x46, 0x46, // ChunkID = "RIFF"
	0x00, 0x00, 0x00, 0x00, // Chunksize (will be overwritten later)
	0x57, 0x41, 0x56, 0x45, // Format = "WAVE"
	0x66, 0x6d, 0x74, 0x20, // Subchunk1ID = "fmt "
	0x10, 0x00, 0x00, 0x00, // Subchunk1Size = 16
	0x01, 0x00, 0x01, 0x00, // AudioFormat = 1 (linear quantization) | NumChannels = 1
	0x80, 0x3e, 0x00, 0x00, // SampleRate = 16000 Hz
	0x00, 0xfa, 0x00, 0x00, // ByteRate = SampleRate * NumChannels * BitsPerSample/8 = 64000
	0x04, 0x00, 0x20, 0x00, // BlockAlign = NumChannels * BitsPerSample/8 = 4 | BitsPerSample = 32
	0x64, 0x61, 0x74, 0x61, // Subchunk2ID = "data"
	0x00, 0x00, 0x00, 0x00, // Subchunk2Size = NumSamples * NumChannels * BitsPerSample / 8 (will be overwritten later)
};

typedef struct {
	FILE* logfiles[4];
	int samples;
} capture;

void in_callback(freenect_device* dev, int num_samples,
                 int32_t* mic1, int32_t* mic2,
                 int32_t* mic3, int32_t* mic4,
                 int16_t* cancelled, void *unknown) {
	capture* c = (capture*)freenect_get_user(dev);
	fwrite(mic1, 1, num_samples*sizeof(int32_t), c->logfiles[0]);
	fwrite(mic2, 1, num_samples*sizeof(int32_t), c->logfiles[1]);
	fwrite(mic3, 1, num_samples*sizeof(int32_t), c->logfiles[2]);
	fwrite(mic4, 1, num_samples*sizeof(int32_t), c->logfiles[3]);
	c->samples += num_samples;
	printf("Sample received.  Total samples recorded: %d\n", c->samples);
}

void cleanup(int sig) {
	printf("Caught SIGINT, cleaning up\n");
	die = 1;
}

// GLPC part:
int window;
GLuint gl_rgb_tex;
int mx=-1,my=-1;        // Prevous mouse coordinates
int rotangles[2] = {0}; // Panning angles
float zoom = 1;         // zoom factor
int color = 1;          // Use the RGB texture or just draw it as color

// Do the projection from u,v,depth to X,Y,Z directly in an opengl matrix
// These numbers come from a combination of the ros kinect_node wiki, and
// nicolas burrus' posts.
void LoadVertexMatrix()
{
    float fx = 594.21f;
    float fy = 591.04f;
    float a = -0.0030711f;
    float b = 3.3309495f;
    float cx = 339.5f;
    float cy = 242.7f;
    GLfloat mat[16] = {
        1/fx,     0,  0, 0,
        0,    -1/fy,  0, 0,
        0,       0,  0, a,
        -cx/fx, cy/fy, -1, b
    };
    glMultMatrixf(mat);
}


// This matrix comes from a combination of nicolas burrus's calibration post
// and some python code I haven't documented yet.
void LoadRGBMatrix()
{
    float mat[16] = {
        5.34866271e+02,   3.89654806e+00,   0.00000000e+00,   1.74704200e-02,
        -4.70724694e+00,  -5.28843603e+02,   0.00000000e+00,  -1.22753400e-02,
        -3.19670762e+02,  -2.60999685e+02,   0.00000000e+00,  -9.99772000e-01,
        -6.98445586e+00,   3.31139785e+00,   0.00000000e+00,   1.09167360e-02
    };
    glMultMatrixf(mat);
}

void mouseMoved(int x, int y)
{
    if (mx>=0 && my>=0) {
        rotangles[0] += y-my;
        rotangles[1] += x-mx;
    }
    mx = x;
    my = y;
}

void mousePress(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        mx = x;
        my = y;
    }
    if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
        mx = -1;
        my = -1;
    }
}

void no_kinect_quit(void)
{
    printf("Error: Kinect not connected?\n");
    exit(1);
}

void DrawGLScene()
{
    short *depth = 0;
    char *rgb = 0;
    uint32_t ts;
    if (freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT) < 0)
	no_kinect_quit();
    if (freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB) < 0)
	no_kinect_quit();

    static unsigned int indices[480][640];
    static short xyz[480][640][3];
    int i,j;
    for (i = 0; i < 480; i++) {
        for (j = 0; j < 640; j++) {
            xyz[i][j][0] = j;
            xyz[i][j][1] = i;
            xyz[i][j][2] = depth[i*640+j];
            indices[i][j] = i*640+j;
        }
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glPushMatrix();
    glScalef(zoom,zoom,1);
    glTranslatef(0,0,-3.5);
    glRotatef(rotangles[0], 1,0,0);
    glRotatef(rotangles[1], 0,1,0);
    glTranslatef(0,0,1.5);

    LoadVertexMatrix();

    // Set the projection from the XYZ to the texture image
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glScalef(1/640.0f,1/480.0f,1);
    LoadRGBMatrix();
    LoadVertexMatrix();
    glMatrixMode(GL_MODELVIEW);

    glPointSize(1);

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_SHORT, 0, xyz);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(3, GL_SHORT, 0, xyz);

    if (color)
        glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb);

    glPointSize(2.0f);
    glDrawElements(GL_POINTS, 640*480, GL_UNSIGNED_INT, indices);
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);
    glutSwapBuffers();
}

void keyPressed(unsigned char key, int x, int y)
{
    if (key == 27) {
        freenect_sync_stop();
        glutDestroyWindow(window);
        exit(0);
    }
    if (key == 'w')
        zoom *= 1.1f;
    if (key == 's')
        zoom /= 1.1f;
    if (key == 'c')
        color = !color;
}

void ReSizeGLScene(int Width, int Height)
{
    glViewport(0,0,Width,Height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, 4/3., 0.3, 200);
    glMatrixMode(GL_MODELVIEW);
}

void InitGL(int Width, int Height)
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glEnable(GL_DEPTH_TEST);
    glGenTextures(1, &gl_rgb_tex);
    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    ReSizeGLScene(Width, Height);
}

// Main has both audio and glpc parts
int main(int argc, char **argv)
{
	pthread_t audio_thread, glpc_thread;

	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}
	freenect_set_log_level(f_ctx, FREENECT_LOG_SPEW);
	freenect_select_subdevices(f_ctx, FREENECT_DEVICE_AUDIO);

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);
	if (nr_devices < 1)
		return 1;

	int user_device_number = 0;
	if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		return 1;
	}

	capture state;
	state.logfiles[0] = fopen("channel1.wav", "wb");
	state.logfiles[1] = fopen("channel2.wav", "wb");
	state.logfiles[2] = fopen("channel3.wav", "wb");
	state.logfiles[3] = fopen("channel4.wav", "wb");
	fwrite(wavheader, 1, 44, state.logfiles[0]);
	fwrite(wavheader, 1, 44, state.logfiles[1]);
	fwrite(wavheader, 1, 44, state.logfiles[2]);
	fwrite(wavheader, 1, 44, state.logfiles[3]);
	freenect_set_user(f_dev, &state);

	freenect_set_audio_in_callback(f_dev, in_callback);
	freenect_start_audio(f_dev);
	signal(SIGINT, cleanup);

	while(!die && freenect_process_events(f_ctx) >= 0) {
		// stick new stuff in here?
	}
		
	glutInit(&argc, argv);

   	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
   	glutInitWindowSize(640, 480);
   	glutInitWindowPosition(0, 0);

 	window = glutCreateWindow("LibFreenect");

	glutDisplayFunc(&DrawGLScene);
	glutIdleFunc(&DrawGLScene);
   	glutReshapeFunc(&ReSizeGLScene);
   	glutKeyboardFunc(&keyPressed);
   	glutMotionFunc(&mouseMoved);
   	glutMouseFunc(&mousePress);

   	InitGL(640, 480);

   	glutMainLoop();

	// Make the WAV header valid for each of the four files
	int i;
	for(i = 0; i < 4 ; i++) {
		char buf[4];
		fseek(state.logfiles[i], 4, SEEK_SET);
		// Write ChunkSize = 36 + subchunk2size
		int chunksize = state.samples * 4 + 36;
		buf[0] = (chunksize & 0x000000ff);
		buf[1] = (chunksize & 0x0000ff00) >> 8;
		buf[2] = (chunksize & 0x00ff0000) >> 16;
		buf[3] = (chunksize & 0xff000000) >> 24;
		fwrite(buf, 1, 4,state.logfiles[i]);

		fseek(state.logfiles[i], 40, SEEK_SET);
		// Write Subchunk2Size = NumSamples * NumChannels (1) * BitsPerSample/8 (4)
		int subchunk2size = state.samples * 4;
		buf[0] = (subchunk2size & 0x000000ff);
		buf[1] = (subchunk2size & 0x0000ff00) >> 8;
		buf[2] = (subchunk2size & 0x00ff0000) >> 16;
		buf[3] = (subchunk2size & 0xff000000) >> 24;
		fwrite(buf, 1, 4,state.logfiles[i]);
		fclose(state.logfiles[i]);
	}
    return 0;
}


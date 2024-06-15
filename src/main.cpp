// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

									// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
	// backspace: reset simulation
	if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
		mj_resetData(m, d);
		mj_forward(m, d);
	}
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
	// update button state
	button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
	button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
	button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

	// update mouse position
	glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
	// no buttons down: nothing to do
	if (!button_left && !button_middle && !button_right) {
		return;
	}

	// compute mouse displacement, save
	double dx = xpos - lastx;
	double dy = ypos - lasty;
	lastx = xpos;
	lasty = ypos;

	// get current window size
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// get shift key state
	bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
		glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

	// determine action based on mouse button
	mjtMouse action;
	if (button_right) {
		action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	}
	else if (button_left) {
		action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	}
	else {
		action = mjMOUSE_ZOOM;
	}

	// move camera
	mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
	// emulate vertical mouse motion = 5% of window height
	mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// main function
int main(int argc, const char** argv) {
	// check command-line arguments
	if (argc != 2) {
		std::printf(" USAGE:  basic modelfile\n");
		return 0;
	}

	// load and compile model
	char error[1000] = "Could not load binary model";
	if (std::strlen(argv[1])>4 && !std::strcmp(argv[1] + std::strlen(argv[1]) - 4, ".mjb")) {
		m = mj_loadModel(argv[1], 0);
	}
	else {
		m = mj_loadXML(argv[1], 0, error, 1000);
	}
	if (!m) {
		mju_error("Load model error: %s", error);
	}

	// make data
	d = mj_makeData(m);

	// init GLFW
	if (!glfwInit()) {
		mju_error("Could not initialize GLFW");
	}

	// create window, make OpenGL context current, request v-sync
	GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// initialize visualization data structures
	mjv_defaultCamera(&cam);
	mjv_defaultOption(&opt);
	mjv_defaultScene(&scn);
	mjr_defaultContext(&con);

	// create scene and context
	mjv_makeScene(m, &scn, 2000);
	mjr_makeContext(m, &con, mjFONTSCALE_150);

	// install GLFW mouse and keyboard callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetScrollCallback(window, scroll);

	 double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 0.000000};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

	// m->opt.gravity[2]=-0.1;
	//qpos is dime nqx1 = 7x1; 3 translations + 4 quaternions
	d->qpos[2] = 0.1;
	d->qvel[2] = 5;
	d->qvel[0] = 1;

	// run main loop, target real-time simulation and 60 fps rendering
	while (!glfwWindowShouldClose(window)) {
		// advance interactive simulation for 1/60 sec
		//  Assuming MuJoCo can simulate faster than real-time, which it usually can,
		//  this loop will finish on time for the next frame to be rendered at 60 fps.
		//  Otherwise add a cpu timer and exit this loop when it is time to render.
		mjtNum simstart = d->time;
		while (d->time - simstart < 1.0 / 60.0) {
			mj_step(m, d);

			double vx, vy, vz;
			vx =d->qvel[0];
			vy =d->qvel[1];
			vz =d->qvel[2];
			double v;
			v=sqrt(vx*vx+vy*vy+vz*vz);

			double fx, fy, fz;
			double c;
			c = 10;

			fx = -c*v*vx;
			fy = -c*v*vy;
			fz = -c*v*vz;

			d->qfrc_applied[0]=fx;
			d->qfrc_applied[1]=fy;
			d->qfrc_applied[2]=fz;

		}

		// get framebuffer viewport
		mjrRect viewport = { 0, 0, 0, 0 };
		glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

		// update scene and render
		opt.frame = mjFRAME_WORLD;
		cam.lookat[0]=d->qpos[0];
		mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);
		// printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

		// swap OpenGL buffers (blocking call due to v-sync)
		glfwSwapBuffers(window);

		// process pending GUI events, call GLFW callbacks
		glfwPollEvents();
	}

	//free visualization storage
	mjv_freeScene(&scn);
	mjr_freeContext(&con);

	// free MuJoCo model and data
	mj_deleteData(d);
	mj_deleteModel(m);

	// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
	glfwTerminate();
#endif

	return 1;
}

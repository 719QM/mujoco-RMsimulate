

#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


char filename[] = "../model/RM75-6F.STEP.SLDASM.xml";

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
bool button_right =  false;
double lastx = 0;
double lasty = 0;
double a0[7]={0}, a1[7]={0}, a2[7]={0}, a3[7]={0};
double qref[7], uref[7];

// state machine
int fsm_state;

#define fsm_hold 0
#define fsm_joint1 1
#define fsm_joint2 2
#define fsm_stop 3

const double t_hold = 0.5;
const double t_joint1 = 1;
const double t_joint2 = 1;


// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}
void set_torque_control(const mjModel* m, int actuator_no, double flag)
{
    if(flag==0)
        m->actuator_gainprm[10*actuator_no+0]=0;
    else
        m->actuator_gainprm[10*actuator_no+0]=1; 
}

void set_position_servo(const mjModel* m, int actuator_no, double kp)
{
    m->actuator_gainprm[10*actuator_no+0]=kp;
    m->actuator_biasprm[10*actuator_no+1]=-kp;
}

void set_velocity_servo(const mjModel* m, int actuator_no, double kv)
{
    m->actuator_gainprm[10*actuator_no+0]=kv;
    m->actuator_biasprm[10*actuator_no+2]=-kv;
}
// *************************
void gengerate_trajectory(double t0, double tf, double q_0[7], double q_f[7])
{
  int i;
  double tf_t0_3 = (tf-t0)*(tf-t0)*(tf-t0);
  for(i=0;i<7;i++)
  {
    double q0 = q_0[i], qf = q_f[i];
    a0[i] = qf*t0*t0*(3*tf-t0) + q0*tf*tf*(tf-3*t0);
    a0[i] = a0[i]/tf_t0_3;
    a1[i] = 6*t0*tf*(q0-qf);
    a1[i] = a1[i]/tf_t0_3;
    a2[i] = 3*(t0+tf)*(qf-q0);
    a2[i] = a2[i]/tf_t0_3;
    a3[i] = 2*(q0-qf);
    a3[i] = a3[i]/tf_t0_3;
  }
}
void init_controller(const mjModel* m, mjData* d)
{
  fsm_state = fsm_hold;
//   set_position_servo( m,1,0); // set pservo1 to 0
//   set_velocity_servo( m,2,0); // set vservo1 to 0
//   set_position_servo( m,4,0); // set pservo2 to 0
//   set_velocity_servo( m,5,0); // set vservo2 to 0

}
void mycontroller(const mjModel* m, mjData* d)
{
    //initial position
    // d->qpos[0] = 0;
    // d->qpos[1] = 0;
    // d->qpos[2] = 0;
    // d->qpos[3] = 0;
    // d->qpos[4] = 0;
    // d->qpos[5] = 0;
    // d->qpos[6] = 0;

      //transitions
  if (fsm_state == fsm_hold && d->time >= t_hold)
  {
    fsm_state = fsm_joint1;
    double q_0[7]={0};
    double q_f[7]={0};
    q_0[0] = 0;
    q_0[1] = 0;
    q_0[2] = 0;
    q_0[3] = 0;
    q_0[4] = 0;
    q_0[5] = 0;
    q_0[6] = 0;

    q_f[0] = 1.57;
    q_f[1] = 0;
    q_f[2] = 0;
    q_f[3] = 0;
    q_f[4] = 0;
    q_f[5] = 0;
    q_f[6] = 0;
    gengerate_trajectory(t_hold, t_hold+t_joint1, q_0,q_f);
  }
  else if (fsm_state == fsm_joint1 && d->time >= (t_hold+t_joint1))
  {
    fsm_state = fsm_joint2;
    double q_0[7]={0};
    double q_f[7]={0};
    q_0[0] = 1.57;
    q_0[1] = 0;
    q_0[2] = 0;
    q_0[3] = 0;
    q_0[4] = 0;
    q_0[5] = 0;
    q_0[6] = 0;

    q_f[0] = 1.57;
    q_f[1] = -1;
    q_f[2] = 0;
    q_f[3] = 0;
    q_f[4] = 0;
    q_f[5] = 1.57;
    q_f[6] = 0;
    gengerate_trajectory(t_hold+t_joint1, t_hold+t_joint1+t_joint2, q_0,q_f);
  }

  else if (fsm_state == fsm_joint2 && d->time>=t_hold+t_joint1+t_joint2)
  {
    fsm_state = fsm_stop;
  }

//   trajectory:
// 1.jonit1: 0->1;
// 2.joint1: 1   ; joint2: 0->1;

double kp = 1000, kv=50;
  if(fsm_state==fsm_hold)
  {
    // q0 = -1; q1 = 0;
     qref[0]=0;   qref[1]=0;    qref[2]=0;  qref[3]=0;  qref[4]=0;  qref[5]=0;  qref[6]=0;
     uref[0]=0;   uref[1]=0;    uref[2]=0;  uref[3]=0;  uref[4]=0;  uref[5]=0;  uref[6]=0;

    d->ctrl[0] = -kp*(d->qpos[0]-qref[0])-kv*d->qvel[0];
    d->ctrl[1] = -kp*(d->qpos[1]-qref[1])-kv*d->qvel[1];
    d->ctrl[2] = -kp*(d->qpos[2]-qref[2])-kv*d->qvel[2];
    d->ctrl[3] = -kp*(d->qpos[3]-qref[3])-kv*d->qvel[3];
    d->ctrl[4] = -kp*(d->qpos[4]-qref[4])-kv*d->qvel[4];
    d->ctrl[5] = -kp*(d->qpos[5]-qref[5])-kv*d->qvel[5];
    d->ctrl[6] = -kp*(d->qpos[6]-qref[6])-kv*d->qvel[6];
    // d->ctrl[1] = q0;
    // d->ctrl[4] = q1;
  }
  else if(fsm_state==fsm_joint1) // generate trajectory
  {
    // q0 = 0; q1 = -1.57;
    // double qref[2]={0}, uref[2]={0};
    for(int i=0;i<7;i++)
    {
      qref[i]=a0[i]+a1[i]*d->time +a2[i]*d->time*d->time + a3[i]*d->time*d->time*d->time;
      uref[i]=      a1[i]   +2*a2[i]*d->time + 3*a3[i]*d->time*d->time;
    }
    d->ctrl[0] = -kp*(d->qpos[0]-qref[0])-kv*(d->qvel[0]-uref[0]);
    d->ctrl[1] = -kp*(d->qpos[1]-qref[1])-kv*(d->qvel[1]-uref[1]);
    d->ctrl[2] = -kp*(d->qpos[2]-qref[2])-kv*(d->qvel[2]-uref[2]);
    d->ctrl[3] = -kp*(d->qpos[3]-qref[3])-kv*(d->qvel[3]-uref[3]);
    d->ctrl[4] = -kp*(d->qpos[4]-qref[4])-kv*(d->qvel[4]-uref[4]);
    d->ctrl[5] = -kp*(d->qpos[5]-qref[5])-kv*(d->qvel[5]-uref[5]);
    d->ctrl[6] = -kp*(d->qpos[6]-qref[6])-kv*(d->qvel[6]-uref[6]);
    
    // d->ctrl[1] = q0;
    // d->ctrl[4] = q1;
  }
  else if(fsm_state==fsm_joint2)  //generate trajectory
  {
    // q0 = 1; q1 = 0;
    // double qref[2]={0}, uref[2]={0};
    for(int i=0;i<7;i++)
    {
      qref[i]=a0[i]+a1[i]*d->time +a2[i]*d->time*d->time + a3[i]*d->time*d->time*d->time;
      uref[i]=      a1[i]   +2*a2[i]*d->time + 3*a3[i]*d->time*d->time;

    }
    d->ctrl[0] = -kp*(d->qpos[0]-qref[0])-kv*(d->qvel[0]-uref[0]);
    d->ctrl[1] = -kp*(d->qpos[1]-qref[1])-kv*(d->qvel[1]-uref[1]);
    d->ctrl[2] = -kp*(d->qpos[2]-qref[2])-kv*(d->qvel[2]-uref[2]);
    d->ctrl[3] = -kp*(d->qpos[3]-qref[3])-kv*(d->qvel[3]-uref[3]);
    d->ctrl[4] = -kp*(d->qpos[4]-qref[4])-kv*(d->qvel[4]-uref[4]);
    d->ctrl[5] = -kp*(d->qpos[5]-qref[5])-kv*(d->qvel[5]-uref[5]);
    d->ctrl[6] = -kp*(d->qpos[6]-qref[6])-kv*(d->qvel[6]-uref[6]);
    // d->ctrl[1] = q0;
    // d->ctrl[4] = q1;
  }
  else if(fsm_state==fsm_stop)
  {

     qref[0]=1.57;   qref[1]=-1;    qref[2]=0;  qref[3]=0;  qref[4]=0;  qref[5]=1.57;  qref[6]=0;
     uref[0]=0;   uref[1]=0;    uref[2]=0;  uref[3]=0;  uref[4]=0;  uref[5]=0;  uref[6]=0;


    d->ctrl[0] = -kp*(d->qpos[0]-qref[0])-kv*d->qvel[0];
    d->ctrl[1] = -kp*(d->qpos[1]-qref[1])-kv*d->qvel[1];
    d->ctrl[2] = -kp*(d->qpos[2]-qref[2])-kv*d->qvel[2];
    d->ctrl[3] = -kp*(d->qpos[3]-qref[3])-kv*d->qvel[3];
    d->ctrl[4] = -kp*(d->qpos[4]-qref[4])-kv*d->qvel[4];
    d->ctrl[5] = -kp*(d->qpos[5]-qref[5])-kv*d->qvel[5];
    d->ctrl[6] = -kp*(d->qpos[6]-qref[6])-kv*d->qvel[6];

    // q0 = 1; q1 = 0;
    // d->ctrl[0] = -kp*(d->qpos[0]-q0)-kv*d->qvel[0];
    // d->ctrl[3] = -kp*(d->qpos[1]-q1)-kv*d->qvel[1];
    // // d->ctrl[1] = q0;
    // d->ctrl[4] = q1;
  }
printf("t: %f       ",d->time);
printf("state: %d \n",fsm_state);

}

// main function
int main(int argc, const char** argv)
{


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {109.028571, -23.257143, 1.074047, -0.021450, -0.028846, 0.380840};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    //initial position
    d->qpos[0] = 0;
    d->qpos[1] = 0;
    d->qpos[2] = 0;
    d->qpos[3] = 0;
    d->qpos[4] = 0;
    d->qpos[5] = 0;
    d->qpos[6] = 0;
    mjcb_control = mycontroller;
    init_controller(m,d);


    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        // printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
